# Plano: adaptar a mímica (head/arm) de K1 pra T2

Pesquisa feita em 2026-08-27, sem acesso físico a um T2 — baseado em specs públicas da Booster (booster.tech, docs.booster.tech, github.com/BoosterRobotics). Blocos marcados **[CONFIRMAR]** só dá pra fechar com acesso real ao robô/docs de dev.

## Diferenças de hardware que mudam o código

| | K1 (atual) | T2 |
|---|---|---|
| DoF total | 22 | 31 |
| DoF por braço | 4 (ombro pitch/roll + cotovelo pitch/yaw) — **sem pulso** | 7 — quase certo que soma pulso roll/pitch/yaw |
| Cintura | nenhuma | 3 DoF (nova) |
| Cabeça | 2 DoF | 2 DoF (igual) |
| Mão/end-effector | opcional no SDK, **não instalada** neste robô | end-effector configurável — **dá pra ter mão/garra destra de verdade** |
| Câmera de pulso | não | opcional |
| Compute | — | Jetson Thor T5000 (Professional) |

## O que precisa mudar em cada peça

### 1. `mimicHead.py`
Baixo risco. A lógica (Face Mesh + solvePnP + `rotate_head` via WS) não depende de DoF de braço/cintura. Só precisa confirmar que o T2 (ou o stack que rodar nele) expõe uma capacidade equivalente a `rotate_head` — **[CONFIRMAR]**.

### 2. `mimicArms.py`
Mudança grande, não só porta. Hoje o código **fixa pulso em 0 de propósito** porque o K1 não tem essa junta:
> "agora também larga qualquer conceito de 'orientação de mão' (roll/pitch/yaw do pulso ficam fixos em 0)"

Pro T2, isso deixa de fazer sentido — o pulso é uma junta real. Precisa:
- Extrair orientação de pulso do vídeo (MediaPipe Pose sozinho não dá isso direito — precisa complementar com MediaPipe Hands, olhando a orientação da palma em relação ao antebraço).
- Mapear pra 3 ângulos novos no protocolo WS (`move_arm` ganha wrist_roll/pitch/yaw, ou um op novo).
- Recalibrar sinais/limites — os `YAW_SIGN`/`PITCH_SIGN` tipo os do `mimicHead.py` vão ter que ser redescobertos ao vivo no T2, do zero (cada robô tem sua própria convenção de sinal, como já vimos com o K1).

### 3. Mão/dedos (capacidade NOVA, não existe hoje)
Se o T2 de vocês vier com a mão destra instalada, dá pra estender a mímica pra dedos — isso conecta direto com o trabalho de CAD da mão que fizemos hoje (`k1-hand/`). Precisaria:
- MediaPipe Hands (21 pontos por mão) em vez de só Pose.
- Descobrir a API de controle de dedo do T2 — **[CONFIRMAR]**, não achei RPC IDs públicos.

### 4. O servidor `duxbot-motion` (lado robô)
Essa é a peça que dá mais trabalho, maior que os scripts do Mac. `duxbot-motion` é código de vocês (não da Booster) que já embrulha o SDK C++ da Booster (`B1LocoClient`) num protocolo WS simples. Pro T2:
- Confirmar se o `booster_robotics_sdk` do T2 ainda usa a classe `B1LocoClient` (mesma do K1/T1) ou se é uma classe nova — os repos públicos oficiais (`booster_robotics_sdk`, `booster_assets`) hoje só documentam **T1 e K1** explicitamente. T2 pode estar numa versão de SDK diferente. **[CONFIRMAR]**
- Se a classe for compatível: portar `motion/adapters/driving/ws_server.py` + `booster_sdk.py` é relativamente mecânico (troca de config/joint names).
- Se não for: reescrever a camada de adapter pro SDK novo do T2.
- Cinemática muda (braço de 7 DoF ≠ 4 DoF) — qualquer lógica de ângulo/limite specific do K1 no `ArmController` precisa ser refeita, não só reaproveitada.

## Ordem sugerida quando o T2 chegar

1. Rodar só `rotate_head`-equivalente no T2 puro (sem DUXBOT), confirmar que dá pra falar com o robô via SDK básico — igual foi feito no K1 originalmente.
2. Portar/adaptar o `duxbot-motion` mínimo (braço sem pulso primeiro, replicando o que já funciona no K1) — prova que a base do stack sobrevive à troca de robô.
3. Só depois, estender pra pulso (3 DoF novos) e, se aplicável, dedos.
4. Cintura (3 DoF) fica de fora do escopo inicial — não tem porquê copiar cintura numa mímica de webcam ainda.

## Bloqueios reais (não dá pra resolver sem o robô)

- Nomes exatos das juntas novas, limites de ângulo, IDs de RPC — só com o SDK/docs de dev do T2 na mão, ou (como rolou com o conector) perguntando direto pro suporte da Booster.
- Convenção de sinal de cada eixo novo — só testando ao vivo.
