#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP-12F 光伏设备 WiFi 桥接 — 本地测试服务端
============================================
在本机启动 TCP 服务器，用于：
  - 接收 ESP-12F 上报的设备状态 JSON
  - 向指定设备发送电机控制命令（除尘/复位）

用法：
  python server_test.py                  # 默认 TCP 模式，端口 9000
  python server_test.py --port 8888      # 指定端口
  python server_test.py --udp            # UDP 模式
  python server_test.py --port 9001 --udp
"""

import socket
import sys
import json
import time
import argparse
import select
import threading
import queue
from datetime import datetime

# ========== 默认配置 ==========
DEFAULT_PORT = 9000
RECV_BUFFER  = 4096

# ========== 颜色输出（Windows 兼容） ==========
try:
    import colorama
    colorama.init()
    class Color:
        GREEN  = '\033[92m'
        YELLOW = '\033[93m'
        CYAN   = '\033[96m'
        RED    = '\033[91m'
        BOLD   = '\033[1m'
        RESET  = '\033[0m'
except ImportError:
    class Color:
        GREEN = YELLOW = CYAN = RED = BOLD = RESET = ''

# ========== 状态缓存 ==========
devices = {}  # dev_id -> 最新状态

# ========== 命令模板 ==========
CMD_TEMPLATES = {
    'clean': '{"cmd":1,"dev":{dev},"act":1}',
    'reset': '{"cmd":1,"dev":{dev},"act":2}',
    'stop':  '{"cmd":1,"dev":{dev},"act":0}',
}

# ========== 帮助信息 ==========
def print_help():
    print(f"""
{Color.BOLD}可用命令:{Color.RESET}
  {Color.GREEN}clean <dev>{Color.RESET}     — 向设备 <dev> 发送除尘命令（如 clean 1）
  {Color.GREEN}reset <dev>{Color.RESET}     — 向设备 <dev> 发送复位命令（如 reset 2）
  {Color.GREEN}stop <dev>{Color.RESET}      — 向设备 <dev> 发送停止命令
  {Color.GREEN}status{Color.RESET}          — 显示所有设备最新状态
  {Color.GREEN}list{Color.RESET}            — 列出已连接的设备
  {Color.GREEN}clear{Color.RESET}           — 清屏
  {Color.GREEN}help{Color.RESET}            — 显示本帮助
  {Color.GREEN}quit / exit{Color.RESET}     — 退出服务端

{Color.BOLD}快捷按键（无需输入命令）:{Color.RESET}
  {Color.GREEN}1c / 2c / 3c / 4c{Color.RESET}  — 对设备 1~4 发送除尘命令
  {Color.GREEN}1r / 2r / 3r / 4r{Color.RESET}  — 对设备 1~4 发送复位命令
  {Color.GREEN}1s / 2s / 3s / 4s{Color.RESET}  — 对设备 1~4 发送停止命令
""")

# ========== 显示设备状态 ==========
def print_status():
    if not devices:
        print(f"  {Color.YELLOW}暂无设备连接{Color.RESET}")
        return

    print(f"\n{Color.BOLD}{'设备':<6} {'模式':<12} {'RSSI':<8} {'最后更新时间':<20}{Color.RESET}")
    print("-" * 56)
    mode_names = {0: '空闲', 1: '清洁中', 2: '复位中', -1: '离线'}
    for dev_id in sorted(devices.keys()):
        d = devices[dev_id]
        mode = d.get('mode', -1)
        mode_str = mode_names.get(mode, f'未知({mode})')
        rssi = d.get('rssi', '?')
        last = d.get('last_seen', '?')
        print(f"  {dev_id:<6} {mode_str:<12} {str(rssi):<8} {str(last):<20}")
    print()

# ========== 列出已连接设备 ==========
def print_list():
    if not devices:
        print(f"  {Color.YELLOW}暂无设备连接{Color.RESET}")
        return
    print(f"\n  已连接设备: {', '.join(f'#{i}' for i in sorted(devices.keys()))}")
    print()

# ========== 解析收到的 JSON ==========
def handle_received(data_str, addr=None):
    try:
        data = json.loads(data_str.strip())
    except json.JSONDecodeError:
        print(f"{Color.RED}[收到非 JSON 数据]{Color.RESET} {addr or ''}: {data_str.strip()}")
        return

    dev_id = data.get('dev', '?')
    mode = data.get('mode', -1)
    rssi = data.get('rssi', '?')
    ts   = data.get('ts', 0)

    mode_names = {0: '空闲', 1: '清洁中', 2: '复位中', -1: '未知'}
    mode_str = mode_names.get(mode, f'未知({mode})')

    # 更新缓存
    devices[dev_id] = {
        'mode': mode,
        'rssi': rssi,
        'ts': ts,
        'last_seen': datetime.now().strftime('%H:%M:%S'),
    }

    addr_str = f' [{addr[0]}:{addr[1]}]' if addr else ''
    print(f"{Color.CYAN}[状态上报{addr_str}]{Color.RESET} 设备#{dev_id} → {mode_str}  RSSI={rssi}")

# ========== TCP 服务端 ==========
class TCPServer:
    def __init__(self, host='0.0.0.0', port=DEFAULT_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.clients = {}   # fd -> (socket, address)
        self.running = False

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)
        self.sock.setblocking(False)
        self.running = True
        print(f"{Color.GREEN}[TCP 服务端启动]{Color.RESET} 监听 {self.host}:{self.port}")
        print(f"  等待 ESP-12F 设备连接...\n")

    def stop(self):
        self.running = False
        for fd, (client_sock, addr) in list(self.clients.items()):
            try:
                client_sock.close()
            except Exception:
                pass
        self.clients.clear()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        print(f"{Color.YELLOW}[TCP 服务端已关闭]{Color.RESET}")

    def send_to_client(self, client_sock, data):
        try:
            client_sock.sendall((data + '\n').encode('utf-8'))
            return True
        except Exception as e:
            print(f"{Color.RED}[发送失败]{Color.RESET} {e}")
            return False

    def send_to_all(self, data):
        for fd, (client_sock, addr) in list(self.clients.items()):
            if not self.send_to_client(client_sock, data):
                self._remove_client(fd)

    def send_to_device(self, dev_id, data):
        """根据设备 ID 发送（如果知道是哪个连接）"""
        # 广播给所有连接（ESP 会根据 deviceID 自行过滤）
        self.send_to_all(data)

    def _remove_client(self, fd):
        if fd in self.clients:
            sock, addr = self.clients.pop(fd)
            try:
                sock.close()
            except Exception:
                pass
            print(f"{Color.YELLOW}[设备断开]{Color.RESET} {addr[0]}:{addr[1]}")

    def fileno(self):
        return self.sock.fileno() if self.sock else -1

    def handle_accept(self):
        try:
            client_sock, addr = self.sock.accept()
            client_sock.setblocking(False)
            fd = client_sock.fileno()
            self.clients[fd] = (client_sock, addr)
            print(f"{Color.GREEN}[设备连接]{Color.RESET} {addr[0]}:{addr[1]}")
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"{Color.RED}[接受连接错误]{Color.RESET} {e}")

    def handle_read(self, fd):
        if fd not in self.clients:
            return
        client_sock, addr = self.clients[fd]
        try:
            raw = client_sock.recv(RECV_BUFFER)
            if not raw:
                self._remove_client(fd)
                return
            # 可能一次收到多行
            for line in raw.decode('utf-8', errors='replace').split('\n'):
                line = line.strip()
                if line:
                    handle_received(line, addr)
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"{Color.RED}[读取错误]{Color.RESET} {addr}: {e}")
            self._remove_client(fd)

# ========== UDP 服务端 ==========
class UDPServer:
    def __init__(self, host='0.0.0.0', port=DEFAULT_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.last_client = None  # 记住最后一个发来数据的客户端地址

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)
        print(f"{Color.GREEN}[UDP 服务端启动]{Color.RESET} 监听 {self.host}:{self.port}")
        print(f"  等待 ESP-12F 设备数据...\n")

    def stop(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        print(f"{Color.YELLOW}[UDP 服务端已关闭]{Color.RESET}")

    def send_to_all(self, data):
        """UDP 广播给最近通信的客户端"""
        if not self.last_client:
            print(f"{Color.YELLOW}[UDP] 无目标客户端（尚未收到任何设备数据）{Color.RESET}")
            return
        try:
            self.sock.sendto((data + '\n').encode('utf-8'), self.last_client)
        except Exception as e:
            print(f"{Color.RED}[UDP 发送失败]{Color.RESET} {e}")

    def send_to_device(self, dev_id, data):
        self.send_to_all(data)

    def fileno(self):
        return self.sock.fileno() if self.sock else -1

    def handle_read(self, fd):
        try:
            raw, addr = self.sock.recvfrom(RECV_BUFFER)
            self.last_client = addr
            for line in raw.decode('utf-8', errors='replace').split('\n'):
                line = line.strip()
                if line:
                    handle_received(line, addr)
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"{Color.RED}[UDP 读取错误]{Color.RESET} {e}")

# ========== 快捷命令处理 ==========
def handle_shortcut(cmd, server):
    """处理单字符快捷命令如 1c, 2r, 3s"""
    if len(cmd) < 2:
        return False
    try:
        dev = int(cmd[0])
    except ValueError:
        return False
    if dev < 1 or dev > 4:
        return False

    action_map = {'c': ('clean', '除尘'), 'r': ('reset', '复位'), 's': ('stop', '停止')}
    if cmd[1] not in action_map:
        return False

    act_name, act_cn = action_map[cmd[1]]
    data = CMD_TEMPLATES[act_name].format(dev=dev)
    server.send_to_device(dev, data)
    print(f"  发送 → {act_cn}命令 to 设备#{dev}  {data}")
    return True

# ========== 用户输入处理 ==========
cmd_queue = queue.Queue()

def stdin_reader():
    """后台线程：持续读取 stdin 并把命令放入队列"""
    while True:
        try:
            cmd = sys.stdin.readline()
            if not cmd:  # EOF
                cmd_queue.put(None)
                return
            cmd_queue.put(cmd.strip())
        except Exception:
            cmd_queue.put(None)
            return

def process_user_command(cmd, server):
    """处理一条用户输入命令"""
    if not cmd:
        return

    # 快捷命令
    if handle_shortcut(cmd, server):
        return

    parts = cmd.split()
    action = parts[0].lower()

    if action in ('quit', 'exit', 'q'):
        server.stop()

    elif action == 'help':
        print_help()

    elif action == 'status':
        print_status()

    elif action == 'list':
        print_list()

    elif action == 'clear':
        print('\n' * 50)

    elif action in ('clean', 'reset', 'stop'):
        dev = parts[1] if len(parts) > 1 else None
        if not dev:
            print(f"  {Color.RED}用法: {action} <设备号 1-4>{Color.RESET}")
            return
        try:
            dev = int(dev)
            if dev < 1 or dev > 4:
                raise ValueError
        except ValueError:
            print(f"  {Color.RED}设备号需为 1~4{Color.RESET}")
            return
        data = CMD_TEMPLATES[action].format(dev=dev)
        server.send_to_device(dev, data)
        act_names = {'clean': '除尘', 'reset': '复位', 'stop': '停止'}
        print(f"  发送 → {act_names[action]}命令 to 设备#{dev}  {data}")

    else:
        print(f"  {Color.RED}未知命令: {cmd}{Color.RESET}")
        print(f"  {Color.YELLOW}输入 help 查看可用命令{Color.RESET}")

# ========== 交互式命令循环 ==========
def run_interactive(server):
    # 启动后台线程读取 stdin（Windows 上 select 不能监听 stdin）
    reader_thread = threading.Thread(target=stdin_reader, daemon=True)
    reader_thread.start()

    print_help()
    print(f"  输入命令后按回车发送。{Color.BOLD}输入 help 查看更多。{Color.RESET}\n")

    while server.running:
        try:
            # 只用 select 监听 socket（Windows 安全）
            readable, _, _ = select.select([server], [], [], 0.3)

            for r in readable:
                if isinstance(server, TCPServer):
                    server.handle_accept()
                    for fd in list(server.clients.keys()):
                        server.handle_read(fd)
                elif isinstance(server, UDPServer):
                    server.handle_read(server.fileno())

            # 处理队列中的用户命令
            while not cmd_queue.empty():
                cmd = cmd_queue.get_nowait()
                if cmd is None:  # stdin EOF
                    server.stop()
                    return
                process_user_command(cmd, server)

        except KeyboardInterrupt:
            print(f"\n{Color.YELLOW}收到中断信号，正在关闭...{Color.RESET}")
            server.stop()
            return
        except OSError as e:
            if e.winerror == 10038:  # 非套接字操作，忽略
                pass
            else:
                raise
        except Exception as e:
            print(f"{Color.RED}运行错误: {e}{Color.RESET}")

# ========== 主入口 ==========
def main():
    parser = argparse.ArgumentParser(
        description='ESP-12F 光伏设备 WiFi 桥接 — 本地测试服务端',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python server_test.py                     # TCP 模式，端口 9000
  python server_test.py --port 8888         # TCP 模式，端口 8888
  python server_test.py --udp               # UDP 模式
  python server_test.py --udp --port 9001   # UDP 模式，端口 9001
        """
    )
    parser.add_argument('-p', '--port', type=int, default=DEFAULT_PORT,
                        help=f'监听端口（默认 {DEFAULT_PORT}）')
    parser.add_argument('-u', '--udp', action='store_true',
                        help='使用 UDP 模式（默认 TCP）')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                        help='绑定地址（默认 0.0.0.0）')
    args = parser.parse_args()

    print(f"{Color.BOLD}ESP-12F 光伏设备 WiFi 桥接 — 本地测试服务端{Color.RESET}")
    print(f"  协议: {'UDP' if args.udp else 'TCP'}")
    print(f"  端口: {args.port}")
    print()

    if args.udp:
        server = UDPServer(args.host, args.port)
    else:
        server = TCPServer(args.host, args.port)

    server.start()
    run_interactive(server)


if __name__ == '__main__':
    main()
