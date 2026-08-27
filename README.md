# K1 Movement Mimicry

Webcam (Mac) → pose estimation → WebSocket para o robô K1 (DUXBOT stack), pra fazer o robô imitar movimentos de cabeça/braço em tempo real.

## Status (2026-08-22)

- **`mimicHead.py` — funcionando.** v2 usa MediaPipe Face Mesh (6 pontos) + `cv2.solvePnP` pra estimar orientação 3D real da cabeça (não só posição), manda via WS `rotate_head` pro `duxbot-motion` (porta `:8765`). Essa capacidade já existe e está comprovada no robô — nenhum código novo do lado do robô foi necessário.
- **`mimicArms.py` — pronto no lado Mac, bloqueado no lado robô.** Fala um protocolo novo (`switch_arm_control` / `move_arm`) que o `duxbot-motion` (`/opt/duxbot-local/motion` no robô) ainda **não implementa** — precisa do deploy correspondente lá antes de funcionar. Além disso, hoje o `duxbot-motion` só escuta em `127.0.0.1:8765` no robô (não `0.0.0.0`), então é preciso um túnel SSH pra conectar do Mac:
  ```
  ssh -N -L 8765:127.0.0.1:8765 booster@<ip-do-robo>
  ```
  e rodar com `ROBOT_IP=127.0.0.1`.

Ambos os scripts têm docstrings detalhadas no topo explicando o histórico/decisões — vale ler antes de mexer.

## Dependências

```
pip install -r requirements.txt
```
(`opencv-python`, `mediapipe`, `numpy` — a comunicação WS é feita "na mão" com `socket`/`struct`, sem lib externa de websocket.)

## Uso

```
ROBOT_IP=<ip-do-robo> python3 mimicHead.py
```
ESC fecha a janela e centraliza a cabeça antes de sair.

## Contexto do robô (pra quem for mexer)

- Robô: Booster K1 humanoide, rodando "DUXBOT" (stack de voz próprio) — não é o firmware padrão da Booster sozinho.
- `duxbot-motion.service` no robô expõe gestos/movimento via WebSocket `:8765`.
- **Gotcha crítico**: o cliente (Mac) precisa disparar e esquecer (fire-and-forget) — não esperar a resposta do WS no loop de vídeo, senão trava o feed da webcam inteiro toda vez que o robô demora pra responder.
- O braço usa `B1LocoClient.MoveHandEndEffector` (posição cartesiana do efetuador, não dedos — o K1 não tem mão/gripper instalado neste robô) precedido por `SwitchHandEndEffectorControlMode`.
