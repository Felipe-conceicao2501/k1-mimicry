"""Mimetismo de cabeça: webcam do Mac -> MediaPipe Face Mesh + solvePnP -> WS
rotate_head no robô.

v2 (2026-08-12): a v1 usava só a posição do nariz relativa ao ombro (MediaPipe Pose) —
um proxy FRACO pra rotação da cabeça, porque girar só a cabeça (sem girar o corpo)
desloca o nariz muito pouco na imagem, mesmo numa virada grande. Resultado: "micro
movimentos", não acompanhava giros de verdade (relatado pelo usuário em teste ao vivo).

v2 estima a ORIENTAÇÃO 3D real da cabeça: 6 pontos-chave do rosto (MediaPipe Face Mesh)
+ um modelo 3D genérico de face + cv2.solvePnP (técnica clássica de head pose
estimation, mesma usada em tutoriais de OpenCV/dlib). Isso mede ROTAÇÃO diretamente,
não uma translação de proxy — muito mais sensível a giros reais da cabeça.

Manda via WS pro duxbot-motion (:8765) usando "rotate_head" — capacidade JÁ existente
e comprovada no robô (não precisou de nenhum código novo do lado do robô).

Uso:
  ROBOT_IP=192.168.0.62 python3 mimicHead.py

ESC fecha a janela e centraliza a cabeça (yaw=0, pitch=0) antes de sair.

Sinal/direção: solvePnP não garante que o sinal do yaw/pitch bata com a convenção do
robô de cara — YAW_SIGN/PITCH_SIGN abaixo existem pra inverter rápido se o robô girar
pro lado errado no primeiro teste real (visto e ajustado ao vivo).
"""
from __future__ import annotations

import json
import os
import select
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

# Envelope conservador (dentro do provado no CLI de exemplo do SDK: yaw ate ~0.785,
# pitch -0.3..1.0) — aqui mais contido pra mimetismo suave.
YAW_MAX = 0.7
PITCH_MIN, PITCH_MAX = -0.25, 0.45
SEND_HZ = 8.0
YAW_SIGN = 1.0
PITCH_SIGN = 1.0

# Estabilidade do solvePnP: com so 6 pontos e sem continuidade entre frames, a pose
# pode "pular" pra uma solucao alternativa ambigua a cada frame (visto ao vivo:
# pitch=79 do nada). Duas defesas: (1) usa a estimativa do frame anterior como chute
# inicial (useExtrinsicGuess) — o solver tende a ficar perto da solucao anterior em vez
# de pular pra outra; (2) EMA + limite de salto maximo por atualizacao nos angulos
# finais, filtrando o que ainda passar.
SMOOTH_ALPHA = 0.35       # EMA: menor = mais suave/mais atraso
MAX_DELTA_DEG = 15.0      # salto maximo aceito por atualizacao (~a cada 1/SEND_HZ s)

# Modelo 3D generico de face (unidades arbitrarias, mesma referencia usada em
# tutoriais classicos de head pose estimation com OpenCV/dlib) — ordem tem que bater
# com FACE_LANDMARK_IDX abaixo.
MODEL_POINTS = np.array([
    (0.0, 0.0, 0.0),          # ponta do nariz
    (0.0, -330.0, -65.0),     # queixo
    (-225.0, 170.0, -135.0),  # canto externo do olho esquerdo (da pessoa)
    (225.0, 170.0, -135.0),   # canto externo do olho direito (da pessoa)
    (-150.0, -150.0, -125.0), # canto da boca esquerdo
    (150.0, -150.0, -125.0),  # canto da boca direito
], dtype="double")

# Indices no topology do MediaPipe Face Mesh (468/478 pontos) correspondentes, na
# MESMA ordem do MODEL_POINTS acima.
FACE_LANDMARK_IDX = [1, 152, 33, 263, 61, 291]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WSClient:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = "duxbot-mimic-head-key=="
        req = (
            f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode())
        resp = self.sock.recv(4096)
        if b"101" not in resp:
            raise RuntimeError(f"handshake WS falhou: {resp!r}")
        self.sock.settimeout(2.0)

    def send(self, msg: dict) -> dict | None:
        data = json.dumps(msg).encode()
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        n = len(data)
        if n < 126:
            hdr = struct.pack(">BB", 0x81, 0x80 | n)
        else:
            hdr = struct.pack(">BBH", 0x81, 0x80 | 126, n)
        self.sock.sendall(hdr + mask + masked)
        try:
            hdr2 = self.sock.recv(2)
            if not hdr2:
                return None
            ln = hdr2[1] & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", self.sock.recv(2))[0]
            payload = b""
            while len(payload) < ln:
                chunk = self.sock.recv(ln - len(payload))
                if not chunk:
                    break
                payload += chunk
            return json.loads(payload.decode())
        except socket.timeout:
            return None

    def send_nowait(self, msg: dict) -> None:
        """Manda sem esperar resposta — o robô hoje está com RPC lento (visto em
        sessão de testes 2026-08-12), então esperar a resposta no loop de vídeo
        trava a imagem inteira. Fogo-e-esquece: qualquer reply que chegar depois
        fica boiando no socket buffer e é descartado no próximo send()/close()."""
        data = json.dumps(msg).encode()
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        n = len(data)
        if n < 126:
            hdr = struct.pack(">BB", 0x81, 0x80 | n)
        else:
            hdr = struct.pack(">BBH", 0x81, 0x80 | 126, n)
        try:
            self.sock.sendall(hdr + mask + masked)
        except OSError as e:
            print(f"[mimic-head] send_nowait falhou: {e}")

    def drain(self) -> None:
        """Descarta qualquer resposta parada no buffer (best-effort, nao-bloqueante) —
        chamar periodicamente enquanto usa send_nowait, senao o buffer cresce sem
        controle numa sessao longa."""
        try:
            while select.select([self.sock], [], [], 0)[0]:
                if not self.sock.recv(4096):
                    break
        except OSError:
            pass

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    host = os.environ.get("ROBOT_IP")
    if not host:
        print("Defina ROBOT_IP (ex.: ROBOT_IP=192.168.0.62 python3 mimicHead.py)")
        sys.exit(1)
    port = int(os.environ.get("ROBOT_MOTION_PORT", "8765"))

    print(f"[mimic-head] conectando ws://{host}:{port} ...", flush=True)
    ws = WSClient(host, port)
    print("[mimic-head] conectado. ESC na janela da webcam pra sair.", flush=True)

    mp_face = mp.solutions.face_mesh

    cap = cv2.VideoCapture(0)
    last_send = 0.0
    send_interval = 1.0 / SEND_HZ
    frame_w, frame_h = None, None
    camera_matrix, dist_coeffs = None, np.zeros((4, 1))
    prev_rvec, prev_tvec = None, None
    smooth_yaw_deg, smooth_pitch_deg = None, None

    try:
        with mp_face.FaceMesh(max_num_faces=1, refine_landmarks=True,
                              min_detection_confidence=0.6,
                              min_tracking_confidence=0.6) as face_mesh:
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    continue

                if frame_w is None:
                    frame_h, frame_w = frame.shape[:2]
                    focal = frame_w
                    center = (frame_w / 2, frame_h / 2)
                    camera_matrix = np.array([
                        [focal, 0, center[0]],
                        [0, focal, center[1]],
                        [0, 0, 1],
                    ], dtype="double")

                frame.flags.writeable = False
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_mesh.process(rgb)
                frame.flags.writeable = True

                now = time.monotonic()
                due = (now - last_send) >= send_interval

                if results.multi_face_landmarks:
                    face_lm = results.multi_face_landmarks[0].landmark
                    # pontinhos de debug nos 6 landmarks usados (feedback visual de que
                    # a deteccao esta pegando os pontos certos).
                    for idx in FACE_LANDMARK_IDX:
                        p = face_lm[idx]
                        cv2.circle(frame, (int(p.x * frame_w), int(p.y * frame_h)), 3, (0, 255, 0), -1)

                    if due:
                        image_points = np.array([
                            (face_lm[idx].x * frame_w, face_lm[idx].y * frame_h)
                            for idx in FACE_LANDMARK_IDX
                        ], dtype="double")

                        ok_pnp, rvec, tvec = cv2.solvePnP(
                            MODEL_POINTS, image_points, camera_matrix, dist_coeffs,
                            rvec=prev_rvec, tvec=prev_tvec,
                            useExtrinsicGuess=prev_rvec is not None,
                            flags=cv2.SOLVEPNP_ITERATIVE)

                        if ok_pnp:
                            last_send = now
                            prev_rvec, prev_tvec = rvec, tvec
                            rot_mat, _ = cv2.Rodrigues(rvec)
                            pose_mat = cv2.hconcat((rot_mat, np.zeros((3, 1))))
                            _, _, _, _, _, _, euler = cv2.decomposeProjectionMatrix(pose_mat)
                            pitch_deg, yaw_deg, _roll_deg = euler.flatten()
                            # decomposeProjectionMatrix pode devolver o pitch "invertido"
                            # (>90 ou <-90) por ambiguidade de sinal do solvePnP — normaliza
                            # pro intervalo -90..90 antes de converter.
                            if pitch_deg > 90:
                                pitch_deg -= 180
                            elif pitch_deg < -90:
                                pitch_deg += 180

                            # limite de salto + EMA — filtra os picos que passarem do
                            # useExtrinsicGuess (ruido de landmark ainda existe).
                            if smooth_yaw_deg is None:
                                smooth_yaw_deg, smooth_pitch_deg = yaw_deg, pitch_deg
                            else:
                                yaw_deg = smooth_yaw_deg + clamp(
                                    yaw_deg - smooth_yaw_deg, -MAX_DELTA_DEG, MAX_DELTA_DEG)
                                pitch_deg = smooth_pitch_deg + clamp(
                                    pitch_deg - smooth_pitch_deg, -MAX_DELTA_DEG, MAX_DELTA_DEG)
                                smooth_yaw_deg = smooth_yaw_deg * (1 - SMOOTH_ALPHA) + yaw_deg * SMOOTH_ALPHA
                                smooth_pitch_deg = smooth_pitch_deg * (1 - SMOOTH_ALPHA) + pitch_deg * SMOOTH_ALPHA

                            yaw = clamp(YAW_SIGN * np.radians(smooth_yaw_deg) / 1.4, -1.0, 1.0) * YAW_MAX
                            pitch_norm = clamp(PITCH_SIGN * np.radians(smooth_pitch_deg) / 0.8, -1.0, 1.0)
                            pitch = PITCH_MIN + (pitch_norm + 1.0) / 2.0 * (PITCH_MAX - PITCH_MIN)

                            ws.send_nowait({"op": "rotate_head", "pitch": round(pitch, 3),
                                             "yaw": round(yaw, 3)})
                            print(f"[mimic-head] yaw_deg={smooth_yaw_deg:.1f} pitch_deg={smooth_pitch_deg:.1f} "
                                  f"-> yaw={yaw:.2f} pitch={pitch:.2f}", flush=True)

                ws.drain()
                cv2.imshow("Mimic Head (ESC sai)", cv2.flip(frame, 1))
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[mimic-head] centralizando a cabeca antes de sair...")
        try:
            print(ws.send({"op": "rotate_head", "pitch": 0.0, "yaw": 0.0}))
        except Exception as e:  # noqa: BLE001
            print(f"[mimic-head] centralizar falhou (ignorando): {e}")
        ws.close()
        print("[mimic-head] encerrado.")


if __name__ == "__main__":
    main()
