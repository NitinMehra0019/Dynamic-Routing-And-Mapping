# Client
import socket

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('server_ip_address', 8080))

client_socket.sendall(b"Hello from client!")

data = client_socket.recv(1024)
print("Received:", data.decode())

client_socket.close()
