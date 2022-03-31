"""
RaspberryPiに積むコード．Kinectからデータを受け取る．
"""
import socket

# print(socket.gethostbyname(socket.gethostname())) # 

# HOST_NAME = "127.0.0.1" # 127.0.0.1 は自分を表す # raspberrypiのipアドレスを調べて入れる必要あり
HOST_NAME = "157.82.203.53" # raspberrypiのipアドレスを調べて入れる必要あり
PORT = 8080 # 自分で設定するもの．8080はよく使う．(well-known port)

def server_make_connection():
    # ipv4を使うので、AF_INET
    # tcp/ip通信を使いたいので、SOCK_STREAM
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    # localhostとlocal portを指定
    sock.bind((HOST_NAME,PORT))

    # server動作開始
    sock.listen(1)

    # 接続を許可して、待つ
    client,remote_addr = sock.accept()
    print("accepted remote. remote_addr {}.".format(remote_addr))

    return sock, client

def server_receive_massage(sock, client):
    # 接続されたら、データが送られてくるまで待つ
    rcv_data = client.recv(1024)
    # 接続が切られたら、終了
    if not rcv_data:
        print("close client communication")
        server_end(client, sock)
        return 0
    else:
        print(rcv_data.decode("utf-8"))# データはここで取得できる
        client.send(b'OK') # clientにOKを送信
        return 1

def server_end(client, sock):
    # clientとserverのsocketを閉じる
    client.close()
    sock.close()

if __name__ == "__main__":
    sock, client = server_make_connection()
    while True:
        state = server_receive_massage(sock, client)
        if state == 0:
            break