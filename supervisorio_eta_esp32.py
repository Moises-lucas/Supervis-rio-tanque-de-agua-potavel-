"""
=============================================================================
  SUPERVISÓRIO ETA — Estação de Tratamento de Água  (versão ESP32)
  Framework: CustomTkinter
  =============================================================================
  Instalação:
      pip install customtkinter
      pip install pyserial          <-- necessário para comunicação com o ESP32

  Como rodar:
      python supervisorio_eta_esp32.py
=============================================================================

  MODO DE OPERAÇÃO DUPLO
  ----------------------
  Este supervisório funciona em dois modos:

    • SIM  (SIMULAÇÃO) — comportamento 100% idêntico ao original.
                          Você pode testar TODO o funcionamento SEM conectar
                          o ESP32. É o modo padrão ao abrir o programa.

    • HW   (HARDWARE)  — quando você conecta o ESP32 (via porta serial), os
                          valores de nível e vazão passam a vir do hardware
                          real, e os comandos (encher, parar, VCP, etc.) são
                          enviados para o ESP32.

  Ou seja: responde à pergunta do usuário — SIM, é possível fazer toda a
  simulação de teste sem o ESP32 conectado. Basta abrir o programa e usar
  normalmente; ele já inicia em modo SIMULAÇÃO.

  Se o ESP32 for desconectado durante a operação em modo HARDWARE, o
  supervisório volta automaticamente para a SIMULAÇÃO (com aviso no log).

  Protocolo de comunicação (Serial USB, 115200 baud, JSON por linha):
    PC -> ESP32:  {"cmd":"MOTOR","val":1} | {"cmd":"V01","val":1} |
                  {"cmd":"VDRAIN","val":1} | {"cmd":"VCP","val":75} |
                  {"cmd":"SP","val":50} | {"cmd":"PING","val":0}
    ESP32 -> PC:  {"type":"DATA","lit":45.2,"fit1":12.0,"fit2":4.0,"motor":1,
                   "v01":1,"vdrain":0,"vcp":75,"alarm":0,"uptime":12345}
                  {"type":"LOG","msg":"...","level":"ok"}
                  {"type":"PONG","uptime":12345}
=============================================================================

"""

import sys
import math
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import Canvas
from datetime import datetime

try:
    import customtkinter as ctk
except ImportError:
    print("CustomTkinter não encontrado.")
    print("Instale com:  pip install customtkinter")
    sys.exit(1)

# pyserial é opcional para o modo SIMULAÇÃO, mas necessário para o modo HARDWARE.
# Importamos de forma tolerante para que o programa rode mesmo sem a lib instalada
# (nesse caso, apenas o modo SIMULAÇÃO ficará disponível).
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("dark-blue")

# ─── Paleta de cores ───────────────────────────────────────────────────────
C_BG          = "#0d1117"
C_BG_PANEL    = "#0a1410"
C_BG_SECTION  = "#0d1f18"
C_HEADER      = "#0a1f16"
C_BORDER      = "#1e3a2f"
C_GREEN       = "#4ade80"
C_GREEN_DIM   = "#2a5a3a"
C_GREEN_DARK  = "#052e16"
C_BLUE        = "#60a5fa"
C_AMBER       = "#fbbf24"
C_RED         = "#f87171"
C_GRAY        = "#4b7a5a"
C_WATER_TOP   = "#0e7490"
C_WATER_BOT   = "#164e63"
C_ORANGE      = "#fb923c"


# ═════════════════════════════════════════════════════════════════════════════
#  PONTE DE COMUNICAÇÃO COM O ESP32 (EspBridge)
# ═════════════════════════════════════════════════════════════════════════════
class EspBridge:
    """
    Gerencia a comunicação serial com o ESP32 em uma thread separada.

    Responsabilidades:
      • Listar/auto-detectar as portas seriais disponíveis (COMx / ttyUSBx).
      • Conectar e desconectar sob demanda.
      • Ler o JSON enviado pelo ESP32 em uma thread dedicada e depositar cada
        mensagem em uma fila thread-safe (queue.Queue) para a interface consumir.
      • Enviar comandos JSON para o ESP32.
      • Detectar timeout de conexão (sem dados por alguns segundos = offline).

    A interface (App) NÃO acessa a serial diretamente: apenas chama métodos
    desta classe e lê da fila. Assim, a GUI nunca trava esperando I/O.
    """

    # Sem dados por este tempo => consideramos a conexão perdida.
    RX_TIMEOUT_S = 3.0

    def __init__(self, log_callback=None):
        self._log_cb   = log_callback          # função para registrar logs na GUI
        self._ser      = None                  # objeto serial.Serial
        self._thread   = None                  # thread de leitura
        self._running  = False                 # flag de execução da thread
        self.rx_queue  = queue.Queue()         # fila thread-safe de mensagens
        self._lock     = threading.Lock()      # protege o acesso de escrita à serial
        self.port      = None                  # porta atualmente conectada
        self.connected = False                 # True quando a serial está aberta
        self.last_rx   = 0.0                   # timestamp da última recepção
        self.last_uptime = 0                   # último uptime informado pelo ESP32

    # ── Descoberta de portas ────────────────────────────────────────────────
    @staticmethod
    def list_ports():
        """Retorna a lista de nomes de portas seriais disponíveis."""
        if not SERIAL_AVAILABLE:
            return []
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(p.device)
        return ports

    @staticmethod
    def auto_detect():
        """
        Tenta escolher automaticamente a porta mais provável do ESP32.
        Procura por descrições típicas (CP210x, CH340, Silicon Labs, USB...).
        Retorna o nome da porta ou None.
        """
        if not SERIAL_AVAILABLE:
            return None
        candidates = []
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            if any(k in desc or k in hwid for k in
                   ("cp210", "ch340", "ch910", "silicon labs", "usb", "uart", "wch")):
                candidates.append(p.device)
        if candidates:
            return candidates[0]
        # fallback: primeira porta disponível
        allp = EspBridge.list_ports()
        return allp[0] if allp else None

    # ── Conexão / desconexão ────────────────────────────────────────────────
    def connect(self, port, baud=115200):
        """Abre a porta serial e inicia a thread de leitura. Retorna True/False."""
        if not SERIAL_AVAILABLE:
            self._log("pyserial não instalado — rode: pip install pyserial", "alarm")
            return False
        if self.connected:
            self.disconnect()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            # Muitas placas ESP32 resetam ao abrir a serial; aguarda o boot.
            time.sleep(0.3)
            self.port      = port
            self.connected = True
            self.last_rx   = time.time()
            self._running  = True
            self._thread   = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()
            self._log(f"Conectado ao ESP32 em {port} @ {baud}", "ok")
            # Envia um PING inicial para confirmar a comunicação.
            self.send_cmd("PING", 0)
            return True
        except Exception as e:
            self._log(f"Falha ao conectar em {port}: {e}", "alarm")
            self._ser      = None
            self.connected = False
            return False

    def disconnect(self):
        """Encerra a thread de leitura e fecha a porta serial."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser      = None
        self.connected = False
        self.port      = None

    # ── Thread de leitura ────────────────────────────────────────────────────
    def _reader(self):
        """Loop de leitura: lê linhas JSON e as coloca na fila thread-safe."""
        buf = b""
        while self._running and self._ser is not None:
            try:
                data = self._ser.read(256)
                if data:
                    buf += data
                    # Processa todas as linhas completas do buffer.
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            msg = json.loads(line.decode("utf-8", errors="ignore"))
                            self.last_rx = time.time()
                            if isinstance(msg, dict) and "uptime" in msg:
                                self.last_uptime = msg.get("uptime", 0)
                            self.rx_queue.put(msg)
                        except json.JSONDecodeError:
                            # linha corrompida/parcial: ignora
                            pass
                else:
                    time.sleep(0.01)
            except Exception as e:
                # Erro de I/O (cabo removido, etc.) -> sinaliza desconexão.
                self.rx_queue.put({"type": "ERROR", "msg": str(e)})
                self._running  = False
                self.connected = False
                break

    # ── Envio de comandos ────────────────────────────────────────────────────
    def send_cmd(self, cmd, val):
        """
        Envia um comando JSON ao ESP32, ex.: send_cmd("MOTOR", 1).
        Seguro para chamar mesmo desconectado (apenas ignora).
        """
        if not self.connected or self._ser is None:
            return False
        payload = json.dumps({"cmd": cmd, "val": val}) + "\n"
        try:
            with self._lock:
                self._ser.write(payload.encode("utf-8"))
            return True
        except Exception as e:
            self._log(f"Erro ao enviar comando: {e}", "alarm")
            self.connected = False
            return False

    # ── Detecção de timeout ──────────────────────────────────────────────────
    def is_alive(self):
        """
        True se a conexão está aberta E recebemos dados recentemente.
        Se passou do RX_TIMEOUT_S sem dados, consideramos offline.
        """
        if not self.connected:
            return False
        return (time.time() - self.last_rx) < self.RX_TIMEOUT_S

    # ── Log auxiliar ─────────────────────────────────────────────────────────
    def _log(self, msg, kind="info"):
        if self._log_cb is not None:
            self._log_cb(msg, kind)


# ═════════════════════════════════════════════════════════════════════════════
#  CANVAS P&ID
# ═════════════════════════════════════════════════════════════════════════════
class ProcessCanvas(Canvas):
    def __init__(self, master, **kw):
        super().__init__(master, bg=C_BG, highlightthickness=0, **kw)
        self.level        = 13.5
        self.setpoint     = 50.0
        self.filling      = True
        self.motor_on     = True
        self.valve_open   = True
        self.drain_open   = False
        self.maint_mode   = False
        self.vcp_pct      = 50.0   # abertura VCP-01 saída (0-100%)
        self.flow_out     = 0.0    # vazão de saída calculada
        self.flow_in      = 0.0    # vazão de entrada (usada em modo HW)
        self._motor_angle = 0.0
        self.bind("<Configure>", lambda e: self._redraw())

    def set_state(self, level, setpoint, filling, motor_on,
                  valve_open, drain_open, maint_mode, motor_angle,
                  vcp_pct, flow_out, flow_in=None):
        self.level        = level
        self.setpoint     = setpoint
        self.filling      = filling
        self.motor_on     = motor_on
        self.valve_open   = valve_open
        self.drain_open   = drain_open
        self.maint_mode   = maint_mode
        self._motor_angle = motor_angle
        self.vcp_pct      = vcp_pct
        self.flow_out     = flow_out
        if flow_in is not None:
            self.flow_in = flow_in
        self._redraw()

    def _level_color(self):
        if self.level <= 25 and not self.maint_mode:
            return C_RED
        elif self.level <= 35:
            return C_AMBER
        return C_GREEN

    # ── Redesenho ────────────────────────────────────────────────────────
    def _redraw(self):
        self.delete("all")
        W = self.winfo_width();  H = self.winfo_height()
        if W < 10 or H < 10:
            return

        cx = W / 2
        sc = H / 520.0

        def s(v):  return v * sc
        def sx(v): return cx + (v - 200) * sc

        TK_X = sx(130); TK_Y = s(75)
        TK_W = s(140);  TK_H = s(255)
        TK_B = TK_Y + TK_H

        # Tubulação entrada
        self._pipe(sx(195), s(18), s(10), s(62))
        # Tubulação saída (vai para empresa)
        self._pipe(sx(215), TK_B, s(10), s(90), color=C_BLUE)
        # Tubulação dreno
        dc = C_AMBER if self.drain_open else C_BORDER
        self._pipe(sx(161), TK_B, s(10), s(45), color=dc)

        # Tanque
        self._draw_tank(TK_X, TK_Y, TK_W, TK_H, TK_B, sc)

        # Setpoint
        sp_y = TK_B - (self.setpoint / 100.0) * TK_H
        self.create_line(TK_X - s(8), sp_y, TK_X + TK_W + s(8), sp_y,
                         fill=C_BLUE, width=max(1, s(1)), dash=(6, 3))
        self.create_polygon(TK_X-s(14), sp_y-s(4),
                            TK_X-s(6),  sp_y,
                            TK_X-s(14), sp_y+s(4),
                            fill=C_BLUE, outline="")
        self.create_text(TK_X - s(16), sp_y,
                         text=f"SP\n{self.setpoint:.0f}%",
                         fill=C_BLUE,
                         font=("Courier New", max(6, int(s(6)))),
                         anchor="e", justify="right")

        # Motor/Bomba
        self._draw_motor(sx(200), s(34), sc)
        # Válvula entrada
        self._draw_valve_v01(sx(200), s(70), sc)
        # Válvula dreno
        self._draw_drain_valve(sx(166), TK_B + s(10), sc)

        # ── Linha de saída com FIT-02 e VCP-01 ───────────────────────
        out_x = sx(220)
        # FIT-02 (sensor de vazão saída)
        fit2_y = TK_B + s(20)
        self._draw_instrument(out_x + s(14), fit2_y, s(64), s(36),
                              "FIT-02", f"{self.flow_out:.1f} m³/h",
                              sc, C_BLUE)
        # VCP-01 (válvula regulável saída)
        vcp_y = TK_B + s(62)
        self._draw_valve_vcp(out_x, vcp_y, sc, self.vcp_pct)

        # Seta final entrega empresa
        self._flow_arrow(out_x + s(5), vcp_y + s(14),
                         out_x + s(5), vcp_y + s(28), C_BLUE)
        self.create_text(out_x + s(18), vcp_y + s(21),
                         text="EMPRESA", fill=C_BLUE,
                         font=("Courier New", max(6, int(s(6)))),
                         anchor="w")

        # Instrumentos direita
        flow_in = self.flow_in if self.flow_in else (
            12.0 if (self.filling and self.motor_on) else 0.0)
        self._draw_instrument(TK_X + TK_W + s(18), TK_Y + s(45),
                              s(66), s(36), "LIT-01",
                              f"{self.level:.1f}%", sc, self._level_color())
        self._draw_instrument(TK_X + TK_W + s(18), TK_Y + s(105),
                              s(66), s(36), "FIT-01",
                              f"{flow_in:.1f} m³/h", sc,
                              C_GREEN if flow_in > 0 else C_GRAY)

        # Seta entrada
        if self.filling and self.motor_on:
            self._flow_arrow(sx(200), s(16), sx(200), s(64), C_GREEN)

        # Escala %
        for pct, label in [(100,"100%"),(75,"75%"),(50,"50%"),(25,"25%")]:
            ly = TK_B - (pct / 100.0) * TK_H
            c  = C_RED if pct == 25 else C_GREEN_DIM
            self.create_text(TK_X - s(6), ly, text=label, fill=c,
                             font=("Courier New", max(6, int(s(6)))),
                             anchor="e")

    # ── Tubulação ────────────────────────────────────────────────────────
    def _pipe(self, x, y, w, h, color=None):
        c = color or C_BORDER
        self.create_rectangle(x, y, x+w, y+h, fill=c, outline=C_BORDER, width=1)

    # ── Tanque ───────────────────────────────────────────────────────────
    def _draw_tank(self, x, y, w, h, bot, sc):
        self.create_rectangle(x, y, x+w, bot,
                               fill="#040d08", outline="#1e6b3a", width=2)
        water_h = max(2, (self.level / 100.0) * h)
        water_y = bot - water_h
        bands   = max(1, int(water_h / 4))
        for i in range(bands):
            t   = i / max(1, bands-1)
            rc  = int(0x0e + (0x16-0x0e)*t)
            gc  = int(0x74 + (0x4e-0x74)*t)
            bc  = int(0x90 + (0x63-0x90)*t)
            col = f"#{rc:02x}{gc:02x}{bc:02x}"
            self.create_rectangle(x+1, water_y+i*(water_h/bands),
                                  x+w-1, water_y+(i+1)*(water_h/bands),
                                  fill=col, outline="")
        if water_h > 4:
            seg = w / 4
            pts = [x, water_y]
            for i in range(4):
                mid_x = x + i*seg + seg/2
                mid_y = water_y - sc*2.5 if i%2==0 else water_y + sc*2.5
                pts  += [mid_x, mid_y, x+(i+1)*seg, water_y]
            pts += [x+w, bot, x, bot]
            self.create_polygon(*pts, fill=C_WATER_TOP, outline="", smooth=True)

        # Linhas de escala
        for pct in [75, 50]:
            ly = bot - (pct/100.0)*h
            self.create_line(x, ly, x+w, ly, fill=C_BORDER, width=1, dash=(3,4))
        ly25 = bot - 0.25*h
        self.create_line(x, ly25, x+w, ly25, fill=C_RED, width=1, dash=(4,3))

        # Borda
        self.create_rectangle(x, y, x+w, bot, fill="", outline="#1e6b3a", width=2)

        # Display central
        dx = x + w*0.1; dy = y + h*0.28; dw = w*0.8; dh = sc*32
        self.create_rectangle(dx, dy, dx+dw, dy+dh, fill="#040d08", outline="")
        lc = self._level_color()
        self.create_text(dx+dw/2, dy+dh*0.3,
                         text=f"NI: {self.level:.1f}%", fill=lc,
                         font=("Courier New", max(8, int(sc*10)), "bold"),
                         anchor="center")
        vol = (self.level/100.0)*300
        self.create_text(dx+dw/2, dy+dh*0.72,
                         text=f"Vol: {vol:.1f} m³", fill=C_GRAY,
                         font=("Courier New", max(7, int(sc*8))),
                         anchor="center")

        # Barra lateral
        bw = max(4, int(sc*6))
        self.create_rectangle(x-bw-2, y, x-2, bot,
                               fill="#040d08", outline=C_BORDER, width=1)
        bar_h = max(2, (self.level/100.0)*h)
        self.create_rectangle(x-bw-1, bot-bar_h, x-3, bot, fill=lc, outline="")

        # Etiqueta
        self.create_text(x+w/2, bot+sc*10, text="TQ-01  ▪  300 m³",
                         fill=C_GREEN_DIM,
                         font=("Courier New", max(7, int(sc*8))),
                         anchor="center")

        # Banner manutenção
        if self.maint_mode:
            bx = x + w*0.05; by = y + h*0.52
            bw2 = w*0.9;     bh2 = sc*46
            self.create_rectangle(bx, by, bx+bw2, by+bh2,
                                  fill="#1a0e00", outline=C_AMBER, width=1)
            self.create_text(bx+bw2/2, by+bh2*0.22,
                             text="⚙ LIMPEZA INTERNA", fill=C_AMBER,
                             font=("Courier New", max(6, int(sc*7.5)), "bold"),
                             anchor="center")
            self.create_text(bx+bw2/2, by+bh2*0.56,
                             text="Esvaziando para higienização", fill=C_AMBER,
                             font=("Courier New", max(5, int(sc*6.5))),
                             anchor="center")
            self.create_text(bx+bw2/2, by+bh2*0.82,
                             text="Dreno V-DRAIN aberto", fill="#d97706",
                             font=("Courier New", max(5, int(sc*6))),
                             anchor="center")

    # ── Motor / Bomba ────────────────────────────────────────────────────
    def _draw_motor(self, cx, cy, sc):
        rx, ry = sc*22, sc*15
        self.create_oval(cx-rx, cy-ry, cx+rx, cy+ry,
                         fill="#0a1f16", outline="#1e6b3a", width=max(1,int(sc*1.5)))
        self.create_oval(cx-rx*.63, cy-ry*.56, cx+rx*.63, cy+ry*.56,
                         fill="#0d2e1e", outline="#1e6b3a", width=max(1,int(sc)))
        r  = sc*8; a1 = math.radians(self._motor_angle); a2 = a1+2.4
        col = C_GREEN if self.motor_on else C_GRAY
        self.create_polygon(cx, cy,
                            cx+r*math.cos(a1), cy+r*math.sin(a1),
                            cx+r*math.cos(a2), cy+r*math.sin(a2),
                            fill=col, outline="")
        self.create_oval(cx+sc*14, cy-sc*12, cx+sc*22, cy-sc*4, fill=col, outline="")
        self.create_text(cx, cy-ry-sc*10, text="BOMBA B-01", fill=C_GREEN,
                         font=("Courier New", max(6,int(sc*7))), anchor="center")

    # ── Válvula V-01 (entrada) ───────────────────────────────────────────
    def _draw_valve_v01(self, cx, cy, sc):
        vw, vh = sc*32, sc*13
        col = C_GREEN if self.valve_open else C_RED
        self.create_rectangle(cx-vw/2, cy, cx+vw/2, cy+vh,
                               fill="#0a1f16", outline=col, width=max(1,int(sc*1.5)))
        self.create_line(cx, cy-sc*16, cx, cy, fill="#1e6b3a", width=max(1,int(sc)))
        r = sc*8
        self.create_oval(cx-r, cy-sc*15-r, cx+r, cy-sc*15+r,
                         fill="", outline="#1e6b3a", width=max(1,int(sc*1.5)))
        self.create_line(cx-r, cy-sc*15, cx+r, cy-sc*15, fill="#1e6b3a", width=max(1,int(sc)))
        self.create_line(cx, cy-sc*23, cx, cy-sc*7, fill="#1e6b3a", width=max(1,int(sc)))
        lr = sc*3.5
        self.create_oval(cx+sc*14, cy+sc*3, cx+sc*14+lr*2, cy+sc*3+lr*2,
                         fill=col, outline="")
        self.create_text(cx-vw/2-sc*4, cy+vh/2, text="V-01", fill=C_GREEN,
                         font=("Courier New", max(6,int(sc*7))), anchor="e")

    # ── Válvula de Dreno ─────────────────────────────────────────────────
    def _draw_drain_valve(self, cx, cy, sc):
        col = C_AMBER if self.drain_open else "#78350f"
        vw, vh = sc*26, sc*12
        self.create_rectangle(cx-vw/2, cy, cx+vw/2, cy+vh,
                               fill="#0a1f16", outline=col, width=max(1,int(sc*1.5)))
        self.create_line(cx, cy-sc*12, cx, cy, fill=col, width=max(1,int(sc)))
        r = sc*6
        self.create_oval(cx-r, cy-sc*12-r, cx+r, cy-sc*12+r,
                         fill="", outline=col, width=max(1,int(sc*1.5)))
        self.create_line(cx-r, cy-sc*12, cx+r, cy-sc*12, fill=col, width=max(1,int(sc)))
        self.create_line(cx, cy-sc*18, cx, cy-sc*6, fill=col, width=max(1,int(sc)))
        lr = sc*3.5
        self.create_oval(cx+sc*10, cy+sc*3, cx+sc*10+lr*2, cy+sc*3+lr*2,
                         fill=col, outline="")
        self.create_text(cx, cy+vh+sc*10, text="V-DRAIN", fill=col,
                         font=("Courier New", max(6,int(sc*7))), anchor="center")

    # ── Válvula regulável VCP-01 (saída empresa) ─────────────────────────
    def _draw_valve_vcp(self, cx, cy, sc, pct):
        vw, vh = sc*30, sc*14
        # Cor conforme abertura
        col = C_GREEN if pct >= 60 else C_AMBER if pct >= 20 else C_RED

        # Corpo borboleta/globo
        self.create_rectangle(cx-vw/2, cy, cx+vw/2, cy+vh,
                               fill="#0a1f16", outline=col, width=max(1,int(sc*1.5)))

        # Disco interno representando abertura (proporcional)
        inner_w = (pct/100.0) * (vw - sc*4)
        ix = cx - inner_w/2
        self.create_rectangle(ix, cy+sc*3, ix+inner_w, cy+vh-sc*3,
                               fill=col, outline="")

        # Haste e volante
        self.create_line(cx, cy-sc*16, cx, cy, fill=col, width=max(1,int(sc)))
        r = sc*7
        self.create_oval(cx-r, cy-sc*14-r, cx+r, cy-sc*14+r,
                         fill="", outline=col, width=max(1,int(sc*1.5)))
        self.create_line(cx-r, cy-sc*14, cx+r, cy-sc*14, fill=col, width=max(1,int(sc)))
        self.create_line(cx, cy-sc*21, cx, cy-sc*7, fill=col, width=max(1,int(sc)))

        # Percentual de abertura
        self.create_text(cx, cy-sc*14,
                         text=f"{pct:.0f}%", fill=col,
                         font=("Courier New", max(5,int(sc*6.5)), "bold"),
                         anchor="center")

        # Etiqueta
        self.create_text(cx-vw/2-sc*4, cy+vh/2, text="VCP-01", fill=col,
                         font=("Courier New", max(6,int(sc*7))), anchor="e")

    # ── Instrumento genérico ─────────────────────────────────────────────
    def _draw_instrument(self, x, y, w, h, tag, value, sc, color):
        self.create_rectangle(x, y, x+w, y+h,
                               fill="#040d08", outline=C_BORDER, width=1)
        self.create_line(x-w*0.3, y+h/2, x, y+h/2, fill=C_BORDER, width=1, dash=(2,3))
        self.create_text(x+w/2, y+h*0.28, text=tag, fill=C_GRAY,
                         font=("Courier New", max(6,int(sc*7))), anchor="center")
        self.create_text(x+w/2, y+h*0.70, text=value, fill=color,
                         font=("Courier New", max(7,int(sc*9)), "bold"),
                         anchor="center")

    # ── Seta de fluxo ────────────────────────────────────────────────────
    def _flow_arrow(self, x1, y1, x2, y2, color):
        self.create_line(x1, y1, x2, y2-8, fill=color, width=2)
        self.create_polygon(x2-5, y2-8, x2, y2, x2+5, y2-8,
                            fill=color, outline="")


# ═════════════════════════════════════════════════════════════════════════════
#  JANELA PRINCIPAL
# ═════════════════════════════════════════════════════════════════════════════
class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Supervisório ETA — Estação de Tratamento de Água")
        self.geometry("1060x760")
        self.minsize(900, 600)
        self.configure(fg_color=C_BG)

        # ── Modo de operação ─────────────────────────────────────────────
        # "SIM" = simulação (padrão, funciona SEM ESP32)
        # "HW"  = hardware (dados vêm do ESP32 conectado)
        self.mode = "SIM"

        # Ponte de comunicação com o ESP32 (thread serial).
        self.esp = EspBridge(log_callback=self._log_threadsafe)
        self._esp_uptime = 0
        self._esp_was_alive = False   # controla a transição HW -> SIM (fallback)

        # Estado do processo
        self.level        = 13.5
        self.setpoint     = 50.0
        self.filling      = True
        self.motor_on     = True
        self.valve_open   = True
        self.drain_open   = False
        self.maint_mode   = False
        self._motor_angle = 0.0
        self._alarm25     = False
        self.vcp_pct      = 50.0   # abertura válvula saída (%)
        self.flow_out     = 0.0    # vazão de saída m³/h
        self.flow_in_hw   = 0.0    # vazão de entrada vinda do ESP32 (modo HW)

        # Taxas de simulação (% por tick 200 ms)
        self.FILL_RATE    = 0.12
        # DRAIN_RATE removido: nível só cai via VCP-01 ou manutenção
        self.DRAIN_MAINT  = 0.10

        self._build_ui()
        self._update_labels()
        self._refresh_ports()      # popula a lista de portas ao iniciar
        self._tick()

        # Garante o encerramento limpo da thread serial ao fechar a janela.
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ─────────────────────────────────────────────────────────────────────
    #  INTERFACE
    # ─────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Header
        header = ctk.CTkFrame(self, fg_color=C_HEADER, corner_radius=0, height=42)
        header.pack(fill="x", side="top")
        header.pack_propagate(False)
        ctk.CTkLabel(header,
                     text="⬡  ETA SUPERVISÓRIO — ESTAÇÃO DE TRATAMENTO DE ÁGUA",
                     font=ctk.CTkFont("Courier New", 12, weight="bold"),
                     text_color=C_GREEN, fg_color="transparent"
                     ).pack(side="left", padx=16)
        self._status_lbl = ctk.CTkLabel(header, text="● ONLINE",
                                         font=ctk.CTkFont("Courier New", 11),
                                         text_color=C_GREEN, fg_color="transparent")
        self._status_lbl.pack(side="right", padx=8)
        self._clock_lbl = ctk.CTkLabel(header, text="--:--:--",
                                        font=ctk.CTkFont("Courier New", 11),
                                        text_color=C_GRAY, fg_color="transparent")
        self._clock_lbl.pack(side="right", padx=12)
        tk.Frame(self, bg=C_BORDER, height=1).pack(fill="x")

        # Corpo
        body = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=0)
        body.pack(fill="both", expand=True)

        proc_frame = tk.Frame(body, bg=C_BG,
                              highlightbackground=C_BORDER, highlightthickness=1)
        proc_frame.pack(side="left", fill="both", expand=True)
        self._canvas = ProcessCanvas(proc_frame)
        self._canvas.pack(fill="both", expand=True, padx=2, pady=2)

        panel = ctk.CTkScrollableFrame(body, width=282, fg_color=C_BG_PANEL,
                                       corner_radius=0,
                                       scrollbar_button_color=C_BORDER,
                                       scrollbar_button_hover_color=C_GREEN_DIM)
        panel.pack(side="right", fill="y")
        self._build_panel(panel)

    def _build_panel(self, panel):
        # ══════════════════════════════════════════════════════════════════
        # ── Conexão ESP32 (NOVA PRIMEIRA SEÇÃO) ───────────────────────────
        # ══════════════════════════════════════════════════════════════════
        s0 = self._section(panel, "Conexão ESP32")

        # Badge de status da conexão/modo.
        self._conn_badge = ctk.CTkLabel(
            s0, text="⬤ SIMULAÇÃO", height=30,
            font=ctk.CTkFont("Courier New", 11, weight="bold"),
            fg_color=C_GREEN_DARK, text_color=C_GREEN, corner_radius=6
        )
        self._conn_badge.pack(fill="x", pady=(0, 6))

        # Linha: combobox de porta + botão atualizar.
        port_row = ctk.CTkFrame(s0, fg_color="transparent")
        port_row.pack(fill="x", pady=(0, 4))
        self._port_var = tk.StringVar(value="")
        self._port_combo = ctk.CTkComboBox(
            port_row, variable=self._port_var, values=[""],
            font=ctk.CTkFont("Courier New", 10),
            fg_color="#040d08", text_color=C_GREEN,
            border_color=C_BORDER, border_width=1,
            button_color=C_GREEN_DIM, button_hover_color=C_GREEN_DARK,
            dropdown_fg_color="#040d08", dropdown_text_color=C_GREEN,
            width=180
        )
        self._port_combo.pack(side="left", fill="x", expand=True)
        ctk.CTkButton(
            port_row, text="↺", width=34, height=28,
            font=ctk.CTkFont("Courier New", 13, weight="bold"),
            fg_color="#0d1f18", text_color=C_GREEN,
            hover_color=C_GREEN_DARK, border_color=C_BORDER,
            border_width=1, corner_radius=4,
            command=self._refresh_ports
        ).pack(side="right", padx=(4, 0))

        # Botão conectar/desconectar.
        self._btn_conn = ctk.CTkButton(
            s0, text="🔌  CONECTAR ESP32",
            font=ctk.CTkFont("Courier New", 10, weight="bold"),
            fg_color=C_GREEN_DARK, text_color=C_GREEN,
            hover_color="#14532d", border_color="#166534",
            border_width=1, corner_radius=6, height=34,
            command=self._toggle_connection
        )
        self._btn_conn.pack(fill="x", pady=(2, 4))

        # Label de uptime do ESP32.
        self._uptime_lbl = self._metric_row(s0, "Uptime ESP32", "—", C_GRAY)

        if not SERIAL_AVAILABLE:
            ctk.CTkLabel(
                s0,
                text="⚠ pyserial não instalado.\nRode: pip install pyserial\n"
                     "Modo SIMULAÇÃO disponível.",
                font=ctk.CTkFont("Courier New", 9),
                text_color=C_AMBER, fg_color="#1a0e00",
                justify="left", corner_radius=4
            ).pack(fill="x", pady=(4, 0))

        # ── Modo Operação ───────────────────────────────────────────────
        s1 = self._section(panel, "Modo Operação")
        top_row = ctk.CTkFrame(s1, fg_color="transparent")
        top_row.pack(fill="x", pady=(0, 4))
        self._mode_badge = ctk.CTkLabel(top_row, text="AUTO", width=90,
                                         font=ctk.CTkFont("Courier New", 9, weight="bold"),
                                         fg_color=C_GREEN_DARK, text_color=C_GREEN,
                                         corner_radius=4)
        self._mode_badge.pack(side="left")
        self._motor_lbl = ctk.CTkLabel(top_row, text="■ MOTOR ON",
                                        font=ctk.CTkFont("Courier New", 10),
                                        text_color=C_GREEN, fg_color="transparent")
        self._motor_lbl.pack(side="right")
        self._v01_val  = self._metric_row(s1, "Válvula V-01",   "ABERTA",     C_GREEN)
        self._vd_val   = self._metric_row(s1, "V-DRAIN",        "FECHADA",    C_GRAY)
        self._flow_val = self._metric_row(s1, "Vazão entrada",  "12.0 m³/h",  C_GREEN)

        # ── Setpoint ────────────────────────────────────────────────────
        s2 = self._section(panel, "Setpoint de Nível")
        self._sp_var = tk.IntVar(value=50)
        self._sp_spin = ctk.CTkEntry(s2, textvariable=self._sp_var,
                                      font=ctk.CTkFont("Courier New", 16, weight="bold"),
                                      fg_color="#040d08", text_color=C_GREEN,
                                      border_color=C_BORDER, border_width=1,
                                      justify="center", height=38)
        self._sp_spin.pack(fill="x", pady=(0, 2))
        ctk.CTkLabel(s2, text="mín: 25%  —  máx: 100%",
                     font=ctk.CTkFont("Courier New", 9),
                     text_color=C_GRAY, fg_color="transparent").pack()
        ctk.CTkButton(s2, text="▶  APLICAR SP",
                      font=ctk.CTkFont("Courier New", 10, weight="bold"),
                      fg_color=C_GREEN_DARK, text_color=C_GREEN,
                      hover_color="#14532d", border_color="#166534",
                      border_width=1, corner_radius=6, height=34,
                      command=self._apply_setpoint).pack(fill="x", pady=(6, 2))
        self._sp_val = self._metric_row(s2, "SP atual", "50.0%", C_BLUE)

        # ── Válvula Saída (VCP-01) ──────────────────────────────────────
        s_vcp = self._section(panel, "Válvula Saída — VCP-01")

        self._vcp_val = self._metric_row(s_vcp, "Abertura VCP-01", "50%", C_AMBER)
        self._fit2_val = self._metric_row(s_vcp, "FIT-02 Saída", "0.0 m³/h", C_BLUE)

        self._vcp_slider = ctk.CTkSlider(
            s_vcp, from_=0, to=100, number_of_steps=100,
            button_color=C_AMBER, button_hover_color="#d97706",
            progress_color="#78350f", fg_color=C_BORDER,
            command=self._vcp_changed
        )
        self._vcp_slider.set(50)
        self._vcp_slider.pack(fill="x", pady=(6, 2))

        row_btns = ctk.CTkFrame(s_vcp, fg_color="transparent")
        row_btns.pack(fill="x", pady=(4, 0))
        for label, val in [("0%", 0), ("25%", 25), ("50%", 50),
                            ("75%", 75), ("100%", 100)]:
            ctk.CTkButton(row_btns, text=label, width=44, height=26,
                          font=ctk.CTkFont("Courier New", 9, weight="bold"),
                          fg_color="#0d1f18", text_color=C_AMBER,
                          hover_color="#1a1000", border_color="#78350f",
                          border_width=1, corner_radius=4,
                          command=lambda v=val: self._set_vcp(v)
                          ).pack(side="left", padx=1)

        # ── Comandos Manuais ────────────────────────────────────────────
        s3 = self._section(panel, "Comando Manual")
        ctk.CTkButton(s3, text="⬆  ENCHER",
                      font=ctk.CTkFont("Courier New", 10, weight="bold"),
                      fg_color=C_GREEN_DARK, text_color=C_GREEN,
                      hover_color="#14532d", border_color="#166534",
                      border_width=1, corner_radius=6, height=34,
                      command=self._cmd_fill).pack(fill="x", pady=(0, 6))
        ctk.CTkButton(s3, text="■  PARAR ENCHIMENTO",
                      font=ctk.CTkFont("Courier New", 10, weight="bold"),
                      fg_color="#2d0a0a", text_color=C_RED,
                      hover_color="#450a0a", border_color="#7f1d1d",
                      border_width=1, corner_radius=6, height=34,
                      command=self._cmd_stop).pack(fill="x")

        # ── Manutenção ──────────────────────────────────────────────────
        s4 = self._section(panel, "Manutenção — Limpeza do Tanque")

        # Aviso fixo de instrução
        ctk.CTkLabel(s4,
                     text="ℹ  Procedimento: limpeza interna TQ-01\n"
                          "• Feche a entrada de água (V-01)\n"
                          "• Esvazie totalmente pelo V-DRAIN\n"
                          "• Aplique solução desinfetante\n"
                          "• Aguarde 30 min e enxágue\n"
                          "• Desative o modo ao concluir",
                     font=ctk.CTkFont("Courier New", 9),
                     text_color=C_GRAY, fg_color="#060d08",
                     justify="left", anchor="w", corner_radius=4
                     ).pack(fill="x", pady=(0, 6))

        self._btn_maint = ctk.CTkButton(
            s4, text="⚙  INICIAR LIMPEZA INTERNA DO TANQUE",
            font=ctk.CTkFont("Courier New", 10, weight="bold"),
            fg_color="#1a1000", text_color=C_AMBER,
            hover_color="#2d1f00", border_color="#92400e",
            border_width=1, corner_radius=6, height=34,
            command=self._toggle_maint
        )
        self._btn_maint.pack(fill="x")

        self._maint_active_lbl = ctk.CTkLabel(
            s4,
            text="⚠  LIMPEZA EM ANDAMENTO\n"
                 "Dreno V-DRAIN aberto\n"
                 "Nível abaixo de 25% permitido",
            font=ctk.CTkFont("Courier New", 9, weight="bold"),
            text_color=C_AMBER, fg_color="#1a0e00",
            justify="center", corner_radius=4
        )
        # Aparece só quando ativo

        # ── Instrumentação ──────────────────────────────────────────────
        s5 = self._section(panel, "Instrumentação")
        self._lit_val = self._metric_row(s5, "LIT-01 Nível",   "13.5%",    C_GREEN)
        self._vol_val = self._metric_row(s5, "Volume atual",   "40.5 m³",  C_GREEN)
        self._fit1_val = self._metric_row(s5, "FIT-01 Entrada","0.0 m³/h", C_GRAY)
        self._fit2_inst = self._metric_row(s5, "FIT-02 Saída", "0.0 m³/h", C_BLUE)

        # ── Log de Eventos ───────────────────────────────────────────────
        s6 = self._section(panel, "Log de Eventos")
        self._log_box = ctk.CTkTextbox(
            s6, height=130, state="disabled",
            font=ctk.CTkFont("Courier New", 9),
            fg_color="#040d08", text_color=C_GRAY,
            border_width=0, corner_radius=4, wrap="word"
        )
        self._log_box.pack(fill="x")

        # Logs iniciais
        self._log("SISTEMA INICIALIZADO", "info")
        self._log("Modo SIMULAÇÃO ativo — ESP32 não é necessário para testar", "info")
        self._log("Bomba B-01 partida automática", "ok")
        self._log("VCP-01 abertura inicial: 50%", "info")
        self._log("Nível inicial: 13.5% — enchimento em andamento", "warn")

    # ─────────────────────────────────────────────────────────────────────
    #  UTILITÁRIOS
    # ─────────────────────────────────────────────────────────────────────
    def _section(self, parent, title):
        frame = ctk.CTkFrame(parent, fg_color=C_BG_SECTION,
                             corner_radius=8, border_width=1,
                             border_color=C_BORDER)
        frame.pack(fill="x", padx=6, pady=5)
        ctk.CTkLabel(frame, text=f"▸  {title.upper()}",
                     font=ctk.CTkFont("Courier New", 9),
                     text_color=C_GRAY, fg_color="transparent",
                     anchor="w").pack(fill="x", padx=10, pady=(8, 2))
        tk.Frame(frame, bg=C_BORDER, height=1).pack(fill="x", padx=10)
        inner = ctk.CTkFrame(frame, fg_color="transparent")
        inner.pack(fill="x", padx=10, pady=(4, 8))
        return inner

    def _metric_row(self, parent, label, value, color):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=2)
        ctk.CTkLabel(row, text=label,
                     font=ctk.CTkFont("Courier New", 11),
                     text_color=C_GRAY, fg_color="transparent",
                     anchor="w").pack(side="left")
        val_lbl = ctk.CTkLabel(row, text=value,
                               font=ctk.CTkFont("Courier New", 13, weight="bold"),
                               text_color=color, fg_color="transparent",
                               anchor="e")
        val_lbl.pack(side="right")
        return val_lbl

    def _log(self, msg, kind="info"):
        colors = {"alarm": C_RED, "warn": C_AMBER, "ok": C_GREEN, "info": C_BLUE}
        c  = colors.get(kind, C_GRAY)
        ts = datetime.now().strftime("%H:%M:%S")
        self._log_box.configure(state="normal")
        self._log_box.insert("0.0", f"{ts}  {msg}\n")
        lines = self._log_box.get("1.0", "end").splitlines()
        if len(lines) > 40:
            self._log_box.delete(f"{41}.0", "end")
        self._log_box.configure(state="disabled")

    def _log_threadsafe(self, msg, kind="info"):
        """
        Versão do _log segura para ser chamada a partir da thread serial.
        Agenda a escrita no log para rodar na thread principal do Tkinter.
        """
        try:
            self.after(0, lambda: self._log(msg, kind))
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────
    #  CONEXÃO ESP32
    # ─────────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        """Atualiza a lista de portas seriais disponíveis no combobox."""
        ports = EspBridge.list_ports()
        if not ports:
            ports = ["(nenhuma porta)"]
            self._port_combo.configure(values=ports)
            self._port_var.set(ports[0])
            return
        self._port_combo.configure(values=ports)
        # Tenta pré-selecionar a porta mais provável do ESP32.
        auto = EspBridge.auto_detect()
        self._port_var.set(auto if auto in ports else ports[0])
        self._log(f"{len(ports)} porta(s) serial encontrada(s)", "info")

    def _toggle_connection(self):
        """Conecta ou desconecta do ESP32 conforme o estado atual."""
        if self.esp.connected:
            # Desconectar -> volta para SIMULAÇÃO.
            self.esp.disconnect()
            self.mode = "SIM"
            self._esp_was_alive = False
            self._btn_conn.configure(
                text="🔌  CONECTAR ESP32",
                fg_color=C_GREEN_DARK, text_color=C_GREEN,
                border_color="#166534"
            )
            self._set_conn_badge("SIM")
            self._log("ESP32 desconectado — retornando ao modo SIMULAÇÃO", "warn")
            return

        # Conectar.
        port = self._port_var.get()
        if not port or port.startswith("(") or port == "":
            self._log("Selecione uma porta serial válida", "alarm")
            return

        self._set_conn_badge("CONNECTING")
        self.update_idletasks()

        ok = self.esp.connect(port, baud=115200)
        if ok:
            self.mode = "HW"
            self._esp_was_alive = True
            self._btn_conn.configure(
                text="⛔  DESCONECTAR",
                fg_color="#2d0a0a", text_color=C_RED,
                border_color="#7f1d1d"
            )
            self._set_conn_badge("ONLINE")
        else:
            self.mode = "SIM"
            self._set_conn_badge("DISCONNECTED")

    def _set_conn_badge(self, state):
        """Atualiza o badge de status da conexão ESP32."""
        cfg = {
            "SIM":          ("⬤ SIMULAÇÃO",     C_GREEN_DARK, C_GREEN),
            "CONNECTING":   ("⬤ CONECTANDO...", "#422006",    C_AMBER),
            "ONLINE":       ("⬤ ESP32 ONLINE",  C_GREEN_DARK, C_GREEN),
            "DISCONNECTED": ("⬤ DESCONECTADO",  "#2d0a0a",    C_RED),
        }
        text, bg, fg = cfg.get(state, cfg["SIM"])
        self._conn_badge.configure(text=text, fg_color=bg, text_color=fg)
        # Reflete o modo no badge de operação também.
        if state == "ONLINE":
            self._mode_badge.configure(text="HARDWARE", fg_color=C_GREEN_DARK,
                                       text_color=C_BLUE)
        elif state in ("SIM", "DISCONNECTED") and not self.maint_mode:
            self._mode_badge.configure(text="AUTO", fg_color=C_GREEN_DARK,
                                       text_color=C_GREEN)

    def _on_close(self):
        """Encerramento limpo: fecha a serial antes de destruir a janela."""
        try:
            self.esp.disconnect()
        except Exception:
            pass
        self.destroy()

    def _hw(self):
        """Retorna True se estamos operando com o ESP32 realmente vivo."""
        return self.mode == "HW" and self.esp.is_alive()

    # ─────────────────────────────────────────────────────────────────────
    #  COMANDOS
    # ─────────────────────────────────────────────────────────────────────
    def _apply_setpoint(self):
        try:
            v = int(self._sp_spin.get())
        except ValueError:
            self._log("SP inválido! Digite um número entre 25 e 100.", "alarm")
            return
        if v < 25 or v > 100:
            self._log("SP fora da faixa! Permitido: 25–100%", "alarm")
            return
        self.setpoint = float(v)
        self._sp_val.configure(text=f"{self.setpoint:.1f}%")
        self._log(f"Setpoint alterado para {self.setpoint:.1f}%", "info")
        # Modo HW: informa o setpoint ao ESP32.
        if self._hw():
            self.esp.send_cmd("SP", int(self.setpoint))

    def _cmd_fill(self):
        self.filling = self.motor_on = self.valve_open = True
        self._log("Comando: ENCHER acionado", "ok")
        # Modo HW: liga a bomba e abre a válvula de entrada no ESP32.
        if self._hw():
            self.esp.send_cmd("MOTOR", 1)
            self.esp.send_cmd("V01", 1)
        self._update_labels()

    def _cmd_stop(self):
        self.filling = self.motor_on = self.valve_open = False
        self._log("Comando: PARAR enchimento", "warn")
        # Modo HW: desliga a bomba e fecha a válvula de entrada no ESP32.
        if self._hw():
            self.esp.send_cmd("MOTOR", 0)
            self.esp.send_cmd("V01", 0)
        self._update_labels()

    def _vcp_changed(self, val):
        self.vcp_pct = float(val)
        # Modo HW: envia a nova abertura da válvula proporcional ao ESP32.
        if self._hw():
            self.esp.send_cmd("VCP", int(self.vcp_pct))
        self._update_labels()

    def _set_vcp(self, val):
        self.vcp_pct = float(val)
        self._vcp_slider.set(val)
        self._log(f"VCP-01 abertura definida: {val}%", "info")
        # Modo HW: envia a abertura da VCP-01 ao ESP32.
        if self._hw():
            self.esp.send_cmd("VCP", int(val))
        self._update_labels()

    def _toggle_maint(self):
        self.maint_mode = not self.maint_mode
        if self.maint_mode:
            self._btn_maint.configure(
                fg_color="#422006", border_color="#d97706",
                text="⚙  ENCERRAR LIMPEZA DO TANQUE"
            )
            self._maint_active_lbl.pack(fill="x", pady=(6, 0))
            self._mode_badge.configure(text="LIMPEZA",
                                        fg_color="#422006",
                                        text_color=C_AMBER)
            self.drain_open = True
            self.filling    = False
            self.motor_on   = False
            self.valve_open = False
            self._log("MODO LIMPEZA ATIVADO — V-DRAIN aberto / Bomba desligada", "warn")
            self._log("Procedimento: higienização interna TQ-01 iniciada", "info")
            # Modo HW: aciona o dreno e desliga bomba/válvula no ESP32.
            if self._hw():
                self.esp.send_cmd("MOTOR", 0)
                self.esp.send_cmd("V01", 0)
                self.esp.send_cmd("VDRAIN", 1)
        else:
            self._btn_maint.configure(
                fg_color="#1a1000", border_color="#92400e",
                text="⚙  INICIAR LIMPEZA DO TANQUE"
            )
            self._maint_active_lbl.pack_forget()
            # Ao sair da manutenção, respeita o modo atual (HW ou SIM) no badge.
            if self._hw():
                self._mode_badge.configure(text="HARDWARE",
                                            fg_color=C_GREEN_DARK,
                                            text_color=C_BLUE)
            else:
                self._mode_badge.configure(text="AUTO",
                                            fg_color=C_GREEN_DARK,
                                            text_color=C_GREEN)
            self.drain_open = False
            self._log("Limpeza concluída — retornando operação normal", "ok")
            # Modo HW: fecha o dreno no ESP32.
            if self._hw():
                self.esp.send_cmd("VDRAIN", 0)
        self._update_labels()

    # ─────────────────────────────────────────────────────────────────────
    #  ATUALIZAÇÃO DE LABELS
    # ─────────────────────────────────────────────────────────────────────
    def _update_labels(self):
        # Em modo HW, a vazão de entrada vem do ESP32 (FIT-01); em SIM, é fixa.
        if self._hw():
            flow_in = self.flow_in_hw
        else:
            flow_in = 12.0 if (self.filling and self.motor_on) else 0.0
        # Vazão saída proporcional à abertura VCP-01 (em SIM).
        # Em HW, self.flow_out é atualizado a partir do FIT-02 no _tick().
        if not self._hw():
            # Máxima saída: 8 m³/h com VCP 100% aberta
            self.flow_out = (self.vcp_pct / 100.0) * 8.0
        vol  = (self.level / 100.0) * 300.0
        c_lv = (C_RED   if (self.level <= 25 and not self.maint_mode)
                else C_AMBER if self.level <= 35
                else C_GREEN)
        c_vcp = (C_GREEN if self.vcp_pct >= 60
                 else C_AMBER if self.vcp_pct >= 20
                 else C_RED)

        self._lit_val.configure(text=f"{self.level:.1f}%",     text_color=c_lv)
        self._vol_val.configure(text=f"{vol:.1f} m³",          text_color=c_lv)
        self._fit1_val.configure(text=f"{flow_in:.1f} m³/h",
                                 text_color=C_GREEN if flow_in > 0 else C_GRAY)
        self._fit2_inst.configure(text=f"{self.flow_out:.1f} m³/h",
                                  text_color=C_BLUE if self.flow_out > 0 else C_GRAY)
        # No painel VCP
        self._vcp_val.configure(text=f"{self.vcp_pct:.0f}%",   text_color=c_vcp)
        self._fit2_val.configure(text=f"{self.flow_out:.1f} m³/h",
                                 text_color=C_BLUE if self.flow_out > 0 else C_GRAY)
        self._flow_val.configure(text=f"{flow_in:.1f} m³/h",
                                 text_color=C_GREEN if flow_in > 0 else C_GRAY)
        self._v01_val.configure(text="ABERTA"  if self.valve_open else "FECHADA",
                                text_color=C_GREEN if self.valve_open else C_RED)
        self._vd_val.configure(text="ABERTA"   if self.drain_open  else "FECHADA",
                               text_color=C_AMBER if self.drain_open else C_GRAY)
        self._motor_lbl.configure(
            text="■ MOTOR ON" if self.motor_on else "□ MOTOR OFF",
            text_color=C_GREEN if self.motor_on else C_GRAY
        )

    # ─────────────────────────────────────────────────────────────────────
    #  LEITURA DOS DADOS DO ESP32 (modo HW)
    # ─────────────────────────────────────────────────────────────────────
    def _drain_esp_queue(self):
        """
        Consome todas as mensagens pendentes vindas do ESP32.
        Atualiza o estado local (nível, vazões, saídas) com os dados reais.
        Retorna True se recebeu ao menos uma mensagem DATA.
        """
        got_data = False
        while True:
            try:
                msg = self.esp.rx_queue.get_nowait()
            except queue.Empty:
                break

            if not isinstance(msg, dict):
                continue
            mtype = msg.get("type", "")

            if mtype == "DATA":
                got_data = True
                # Atualiza o estado a partir das leituras reais do ESP32.
                self.level      = float(msg.get("lit", self.level))
                self.flow_in_hw = float(msg.get("fit1", 0.0))
                self.flow_out   = float(msg.get("fit2", 0.0))
                self.motor_on   = bool(msg.get("motor", 0))
                self.valve_open = bool(msg.get("v01", 0))
                self.drain_open = bool(msg.get("vdrain", 0))
                self.vcp_pct    = float(msg.get("vcp", self.vcp_pct))
                self._esp_uptime = int(msg.get("uptime", self._esp_uptime))
                # Enchendo se a bomba está ligada e a válvula aberta.
                self.filling = self.motor_on and self.valve_open

            elif mtype == "LOG":
                self._log(f"[ESP32] {msg.get('msg','')}", msg.get("level", "info"))

            elif mtype == "PONG":
                self._esp_uptime = int(msg.get("uptime", self._esp_uptime))

            elif mtype == "ERROR":
                self._log(f"[ESP32] Erro de comunicação: {msg.get('msg','')}", "alarm")

        return got_data

    def _fmt_uptime(self, ms):
        """Formata milissegundos de uptime em HH:MM:SS."""
        s = int(ms // 1000)
        h = s // 3600
        m = (s % 3600) // 60
        sec = s % 60
        return f"{h:02d}:{m:02d}:{sec:02d}"

    # ─────────────────────────────────────────────────────────────────────
    #  TICK — simulação + controle automático  (ou leitura do ESP32)
    # ─────────────────────────────────────────────────────────────────────
    def _tick(self):
        # ══════════════════════════════════════════════════════════════════
        # MODO HARDWARE — lê dados reais do ESP32
        # ══════════════════════════════════════════════════════════════════
        if self.mode == "HW":
            alive = self.esp.is_alive()

            if alive:
                # Consome a fila de mensagens do ESP32 e atualiza o estado.
                self._drain_esp_queue()
                self._uptime_lbl.configure(
                    text=self._fmt_uptime(self._esp_uptime), text_color=C_GREEN)
                self._set_conn_badge("ONLINE")

                # Animação do motor conforme o estado real.
                if self.motor_on:
                    self._motor_angle = (self._motor_angle + 8) % 360

                self._status_lbl.configure(text="● HARDWARE", text_color=C_BLUE)
            else:
                # ── FALLBACK AUTOMÁTICO: ESP32 caiu -> volta para SIMULAÇÃO ──
                if self._esp_was_alive:
                    self._log("ESP32 sem resposta (timeout) — fallback para "
                              "SIMULAÇÃO automática", "alarm")
                self._esp_was_alive = False
                self.mode = "SIM"
                self._set_conn_badge("DISCONNECTED")
                self._uptime_lbl.configure(text="—", text_color=C_GRAY)
                self._btn_conn.configure(
                    text="🔌  CONECTAR ESP32",
                    fg_color=C_GREEN_DARK, text_color=C_GREEN,
                    border_color="#166534"
                )
                # Encerra a serial (que provavelmente já caiu).
                self.esp.disconnect()

        # ══════════════════════════════════════════════════════════════════
        # MODO SIMULAÇÃO — lógica original preservada integralmente
        # ══════════════════════════════════════════════════════════════════
        if self.mode == "SIM":
            if not self.maint_mode:
                # Alarme 25% → enchimento automático
                if self.level <= 25 and not self.filling:
                    self.filling = self.motor_on = self.valve_open = True
                    if not self._alarm25:
                        self._log("ALARME: Nível crítico ≤ 25% — enchimento automático",
                                  "alarm")
                        self._status_lbl.configure(text="● ALARME", text_color=C_RED)
                        self._alarm25 = True
                    self._update_labels()
                # Setpoint atingido → para
                if self.level >= self.setpoint and self.filling:
                    self.filling = self.motor_on = self.valve_open = False
                    if self._alarm25:
                        self._log(f"Setpoint atingido: {self.setpoint:.0f}% — operação normal",
                                  "ok")
                        self._status_lbl.configure(text="● ONLINE", text_color=C_GREEN)
                        self._alarm25 = False
                    self._update_labels()

            # ── Simulação de nível ────────────────────────────────────────
            # O nível SÓ cai por dois motivos:
            #   1. VCP-01 aberta → saída proporcional para a empresa
            #   2. Modo manutenção → dreno V-DRAIN aberto
            # Com VCP-01 em 0% e fora da manutenção, o nível NÃO cai.

            # Saída via VCP-01: máx 8 m³/h com 100% aberta
            # 8 m³/h sobre 300 m³ = 2.667% por hora = 0.000148% por tick (200 ms)
            # Simplificado: 0.044% por tick a 100% (valor perceptível na simulação)
            drain_vcp = (self.vcp_pct / 100.0) * 0.044   # zero quando VCP=0%

            if self.maint_mode:
                # Manutenção: apenas dreno V-DRAIN, VCP não conta
                self.level = max(0.0, self.level - self.DRAIN_MAINT)
            elif self.filling and self.motor_on:
                # Enchendo: entrada menos o que sai pela VCP-01
                self.level = min(100.0, self.level + self.FILL_RATE - drain_vcp)
            else:
                # Parado: nível cai SOMENTE pela VCP-01 (0% = nível estático)
                self.level = max(0.0, self.level - drain_vcp)

            # Animação motor
            if self.motor_on:
                self._motor_angle = (self._motor_angle + 8) % 360

            # Badge de conexão coerente com o modo SIM (se não estiver conectando).
            if not self.esp.connected:
                self._set_conn_badge("SIM")

        # ── Atualização comum de tela (SIM ou HW) ──────────────────────────
        self._clock_lbl.configure(text=datetime.now().strftime("%H:%M:%S"))
        self._update_labels()
        self._canvas.set_state(
            level       = self.level,
            setpoint    = self.setpoint,
            filling     = self.filling,
            motor_on    = self.motor_on,
            valve_open  = self.valve_open,
            drain_open  = self.drain_open,
            maint_mode  = self.maint_mode,
            motor_angle = self._motor_angle,
            vcp_pct     = self.vcp_pct,
            flow_out    = self.flow_out,
            flow_in     = (self.flow_in_hw if self.mode == "HW" else None),
        )
        self.after(200, self._tick)


# ═════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = App()
    app.mainloop()
