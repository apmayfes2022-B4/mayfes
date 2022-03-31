"""
Kinectに積むコード．RaspberryPiにデータを送信する．
"""
import socket

# HOST_NAME = "127.0.0.1" # 127.0.0.1 は自分を表す # raspberrypiのipアドレスを調べて入れる必要あり
HOST_NAME = "157.82.203.53" # raspberrypiのipアドレスを調べて入れる必要あり
PORT = 8080

def client_make_connection():
    # ipv4を使うので、AF_INET
    # tcp/ip通信を使いたいので、SOCK_STREAM
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # サーバーと接続
    sock.connect((HOST_NAME,PORT))
    return sock

def client_send_massage(sock, data):
    # データ送信
    sock.send(data.encode())
    # サーバーからデータ受信
    rcv_data = sock.recv(1024)
    rcv_data = rcv_data.decode('utf-8')
    # 正しく送信できていない場合は警告
    if rcv_data != 'OK':
        print("[Warning] Data was not sent correctly. ")

if __name__ == "__main__":
    sock = client_make_connection()
    client_send_massage(sock, "owaneeee\n")
    client_send_massage(sock, "abe_shinzo\n")
    client_send_massage(sock, "kishida_humio\n")
    client_send_massage(sock, "Yugin_Song\n")
    sock.close()# 通信終了．これ書いとかないと通信終了しなくてだるかったりするので注意．