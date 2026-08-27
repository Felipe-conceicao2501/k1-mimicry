"""Mimetismo de braço: webcam do Mac -> MediaPipe Pose -> WS move_arm no robô.

v2 (2026-08-22, pós-Mapa de Serviços do K1). Reescrito depois de ler o inventário
completo de serviços da plataforma Booster: o K1 NÃO tem junta de mão (gripper/mão
destra são acessórios opcionais do SDK C++, não instalados neste robô) — então este
script nunca tratou dedo/pinça, e agora também larga qualquer conceito de "orientação
de mão" (roll/pitch/yaw do pulso ficam fixos em 0). O foco é só o BRAÇO (ombro+cotovelo)
dos dois lados. Pernas continuam de fora — não é o momento de copiar passada.

Mudou também o protocolo: o servidor atual (`duxbot-motion` no robô, código em
`/opt/duxbot-local/motion`) não tem mais os ops "move_hand"/"release_hand" — foram
removidos numa reescrita do lado do robô. O caminho real disponível no SDK C++ pra
mover o braço é `B1LocoClient.MoveHandEndEffector` (posição cartesiana do efetuador do
braço, não "mão" no sentido de dedos) precedido por `SwitchHandEndEffectorControlMode`.
Este script já fala o protocolo NOVO que o lado do robô vai expor pra isso:

  {"op": "switch_arm_control", "enable": true}   uma vez no início da sessão
  {"op": "move_arm", "hand": 0|1, "x":.., "y":.., "z":.., "time_ms": ..}   por frame
  {"op": "switch_arm_control", "enable": false}  uma vez ao sair

ATENÇÃO — dois pré-requisitos que ainda faltam antes disso funcionar de verdade:
  1) O lado do robô (`motion/adapters/driving/ws_server.py` + `booster_sdk.py`) ainda
     NÃO implementa "switch_arm_control"/"move_arm" — só existem no protocolo deste
     script por enquanto. Precisa do deploy correspondente no robô antes de testar.
  2) O `duxbot-motion` hoje escuta só em 127.0.0.1:8765 no robô (WS_HOST=127.0.0.1 no
     systemd unit), não em 0.0.0.0 — não dá pra conectar direto do Mac pelo IP do robô.
     Rode um túnel SSH antes: `ssh -N -L 8765:127.0.0.1:8765 booster@<ip-do-robo>` e
     use ROBOT_IP=127.0.0.1 aqui (nenhuma mudança no robô é necessária pra isso).

Rastreamento (o que mudou na parte de visão, pra ficar mais ACERTIVO):
  A v1 usava a distância 2D pulso-ombro (em pixels normalizados) como proxy de "quanto
  o braço está esticado pra frente", com uma constante fixa (0.35) de normalização.
  Dois problemas reais: (a) essa distância cresce igual se o braço se estica PRA CIMA,
  PRO LADO ou PRA FRENTE — não distingue direção, só "quão longe o pulso está do
  ombro"; (b) a constante fixa de normalização quebra se a pessoa está mais perto ou
  mais longe da webcam (a mesma extensão de braço vira números bem diferentes).

  v2 usa o ÂNGULO DO COTOVELO (ombro-cotovelo-pulso) como sinal de extensão — invariante
  à escala/distância da câmera (é um ângulo, não uma distância em pixels), e mede
  articulação de verdade em vez de posição bruta. Cotovelo dobrado (~30-40°) = braço
  recolhido; cotovelo esticado (~170-180°) = braço estendido. Pra lateral (Y) e altura
  (Z) do pulso, a posição 2D relativa ao ombro continua sendo o sinal certo, mas agora
  normalizada pelo comprimento do PRÓPRIO braço da pessoa (distância ombro-cotovelo em
  pixels) em vez de uma constante fixa — se auto-calibra ao tamanho/distância da pessoa
  na câmera.

Mapeamento pra pose alvo (x=frente, y=lateral, z=altura, metros, frame do torso do
robô), enviado via "move_arm". Ainda uma APROXIMAÇÃO (não IK real, não usa profundidade
métrica de verdade) — mas mais robusta que a v1. Câmera espelhada na exibição (modo
"espelho": levantar o braço direito levanta o braço direito do robô, do ponto de vista
de quem olha o robô de frente).

Envelope de segurança (cartesiano, não publicado pela plataforma — ver Mapa de Serviços
seção 12, achado "Envelope de segurança duplicado" — isto é SSOT só nosso, o robô clampa
de novo do lado dele como defesa em profundidade):
  x: 0.15 a 0.40 (frente)   y: -0.40 a 0.40 (lateral, + esquerda)   z: -0.15 a 0.35 (altura)

Uso:
  ROBOT_IP=127.0.0.1 python3 mimicArms.py   # com o túnel SSH acima já rodando

ESC fecha a janela e manda "switch_arm_control" com enable=false antes de sair
(desliga o modo de controle do efetuador do braço no robô).
"""
from __future__ import annotations

import json
import math
import os
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp

# ---- envelope de seguranca (SSOT so nosso; o robo clampa de novo do lado dele) ----
X_MIN, X_MAX = 0.15, 0.40
Y_MIN, Y_MAX = -0.40, 0.40
Z_MIN, Z_MAX = -0.15, 0.35
SEND_HZ = 8.0
TIME_MS = 200  # duracao do movimento no robo, um pouco maior que o intervalo de envio
VISIBILITY_MIN = 0.6  # so manda comando se ombro/cotovelo/pulso estao claramente visiveis

# ---- extensao do braco via angulo do cotovelo (graus) ----
ELBOW_BEND_MIN = 35.0   # cotovelo bem dobrado -> extensao 0.0
ELBOW_BEND_MAX = 170.0  # braco esticado -> extensao 1.0

# ---- normalizacao de lateral/altura pelo proprio comprimento do braco (auto-escala) ----
# fator empirico: quantas "distancias ombro-cotovelo" o pulso alcanca lateralmente/em
# altura no limite do envelope (braco bem aberto pro lado ou erguido).
REACH_SCALE = 2.2

# rc do SDK que valem um pequeno backoff (ver Mapa de Servicos secao 8): 429 = chamada
# frequente demais, 409 = transicao de estado em curso. Nao repetimos com os MESMOS
# parametros em 400 (fora de faixa) -- so logamos.
RC_HINTS = {
    429: "chamando rapido demais, atrasando o envio",
    409: "conflito de estado no robo (transicao em curso), tentando de novo em breve",
    400: "parametro/faixa invalida - NAO repetir igual",
    502: "falha de transicao de estado no robo",
    100: "timeout no robo (rede/servico) - verificar duxbot-motion",
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------- cliente WebSocket minimo (stdlib, RFC6455) ----------
class WSClient:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = "duxbot-mimic-arms-key=="
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

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _report_rc(label: str, reply: dict | None) -> None:
    """Loga o rc de forma legivel em vez de so imprimir o dict cru — ver Mapa de
    Servicos secao 8 (achado: codigos de retorno RPC eram descartados sem tratamento)."""
    if reply is None:
        print(f"[mimic] {label}: sem resposta (timeout)")
        return
    if reply.get("ok"):
        return
    rc = reply.get("rc")
    hint = RC_HINTS.get(rc, reply.get("error", "erro desconhecido"))
    print(f"[mimic] {label}: rc={rc} -> {hint}")


def _elbow_angle_deg(shoulder, elbow, wrist) -> float:
    """Angulo em graus no cotovelo, entre os vetores (cotovelo->ombro) e
    (cotovelo->pulso). ~180 = braco esticado, ~30-40 = braco bem dobrado. Invariante a
    escala/distancia da camera (angulo, nao distancia em pixels)."""
    v1x, v1y = shoulder.x - elbow.x, shoulder.y - elbow.y
    v2x, v2y = wrist.x - elbow.x, wrist.y - elbow.y
    n1 = math.hypot(v1x, v1y)
    n2 = math.hypot(v2x, v2y)
    if n1 < 1e-6 or n2 < 1e-6:
        return 90.0
    cos_a = clamp((v1x * v2x + v1y * v2y) / (n1 * n2), -1.0, 1.0)
    return math.degrees(math.acos(cos_a))


def arm_target(shoulder, elbow, wrist) -> tuple[float, float, float]:
    """Mapeia a pose do braco (ombro/cotovelo/pulso, coords normalizadas 0..1 da
    imagem) pra uma pose alvo (x,y,z metros, frame do torso do robo).

    x (frente): extensao do braco via angulo do cotovelo — self-invariante a escala.
    y (lateral), z (altura): posicao do pulso relativa ao ombro, normalizada pelo
    comprimento ombro-cotovelo da propria pessoa (auto-calibra a distancia da camera)."""
    angle = _elbow_angle_deg(shoulder, elbow, wrist)
    extension = clamp((angle - ELBOW_BEND_MIN) / (ELBOW_BEND_MAX - ELBOW_BEND_MIN), 0.0, 1.0)
    x = X_MIN + extension * (X_MAX - X_MIN)

    dx_img = wrist.x - shoulder.x   # + = pulso pra direita da imagem
    dy_img = shoulder.y - wrist.y   # + = pulso ACIMA do ombro (y da imagem cresce pra baixo)
    arm_scale = max(math.hypot(shoulder.x - elbow.x, shoulder.y - elbow.y), 1e-3)

    # lateral (y): camera espelhada na exibicao — dx_img > 0 no frame ORIGINAL (nao
    # espelhado) do mediapipe = pulso pra DIREITA de quem esta na frente da camera =
    # deve virar lateral NEGATIVA (y+ e' esquerda, convencao do robo).
    y_norm = clamp(-dx_img / (arm_scale * REACH_SCALE), -1.0, 1.0)
    y = clamp(y_norm * Y_MAX, Y_MIN, Y_MAX)

    z_norm = clamp(dy_img / (arm_scale * REACH_SCALE), -1.0, 1.0)
    z = Z_MIN + (z_norm + 1.0) / 2.0 * (Z_MAX - Z_MIN)

    return (round(x, 3), round(y, 3), round(z, 3))


def main():
    host = os.environ.get("ROBOT_IP")
    if not host:
        print("Defina ROBOT_IP (ex.: ROBOT_IP=127.0.0.1 python3 mimicArms.py, com o "
              "tunel SSH pro robo ja rodando — ver docstring no topo do arquivo)")
        sys.exit(1)
    port = int(os.environ.get("ROBOT_MOTION_PORT", "8765"))

    print(f"[mimic] conectando ws://{host}:{port} ...")
    ws = WSClient(host, port)
    print("[mimic] conectado. ligando controle do braco (switch_arm_control)...")
    _report_rc("switch_arm_control(true)", ws.send({"op": "switch_arm_control", "enable": True}))
    print("[mimic] ESC na janela da webcam pra sair.")

    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)
    last_send = 0.0
    send_interval = 1.0 / SEND_HZ
    backoff_until = 0.0  # timestamp; enquanto now < isto, pula envio (backoff de 429)

    LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST = 11, 13, 15
    RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST = 12, 14, 16

    try:
        with mp_pose.Pose(model_complexity=1, min_detection_confidence=0.6,
                          min_tracking_confidence=0.6) as pose:
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    print("[mimic] frame vazio, ignorando")
                    continue

                frame.flags.writeable = False
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = pose.process(rgb)
                frame.flags.writeable = True

                now = time.monotonic()
                due = (now - last_send) >= send_interval and now >= backoff_until

                if results.pose_landmarks:
                    mp_drawing.draw_landmarks(
                        frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                    if due:
                        lm = results.pose_landmarks.landmark
                        last_send = now
                        # braco 0 = esquerdo do robo (lado ESQUERDO de quem olha o robo de
                        # frente) = lado DIREITO da pessoa na camera (frame nao espelhado
                        # do mediapipe) -> usa os landmarks RIGHT_* pra hand=0, e vice-versa,
                        # pra virar um espelho de verdade (voce ergue o braco direito, o
                        # robo ergue o braco esquerdo DELE, que fica do seu lado direito
                        # quando ele esta de frente pra voce).
                        triples = [
                            (0, lm[RIGHT_SHOULDER], lm[RIGHT_ELBOW], lm[RIGHT_WRIST]),
                            (1, lm[LEFT_SHOULDER], lm[LEFT_ELBOW], lm[LEFT_WRIST]),
                        ]
                        for hand, shoulder, elbow, wrist in triples:
                            if (shoulder.visibility < VISIBILITY_MIN
                                    or elbow.visibility < VISIBILITY_MIN
                                    or wrist.visibility < VISIBILITY_MIN):
                                continue
                            x, y, z = arm_target(shoulder, elbow, wrist)
                            reply = ws.send({
                                "op": "move_arm", "hand": hand,
                                "x": x, "y": y, "z": z,
                                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                                "time_ms": TIME_MS,
                            })
                            label = "L" if hand == 0 else "R"
                            print(f"[mimic] braco={label} x={x} y={y} z={z} -> {reply}")
                            if reply is not None and not reply.get("ok", False):
                                _report_rc(f"move_arm({label})", reply)
                                if reply.get("rc") == 429:
                                    backoff_until = now + 0.5

                cv2.imshow("Mimic Arms (ESC sai)", cv2.flip(frame, 1))
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[mimic] desligando controle do braco (switch_arm_control)...")
        try:
            _report_rc("switch_arm_control(false)",
                       ws.send({"op": "switch_arm_control", "enable": False}))
        except Exception as e:  # noqa: BLE001
            print(f"[mimic] switch_arm_control(false) falhou (ignorando): {e}")
        ws.close()
        print("[mimic] encerrado.")


if __name__ == "__main__":
    main()
