import socket

HOST_NAME = "127.0.0.1" # 127.0.0.1 は自分を表す # raspberrypiのipアドレスを調べて入れる必要あり
# HOST_NAME = "157.82.203.53"
PORT = 8080

# ipv4を使うので、AF_INET
# tcp/ip通信を使いたいので、SOCK_STREAM
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

# サーバーと接続
sock.connect((HOST_NAME,PORT))

# データ送信
sock.send(b"Hello, TCP")

# サーバーからデータ受信
rcv_data = sock.recv(1024)
print("receive data : {}".format(rcv_data))
sock.close()