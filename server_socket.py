# Server
import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8080))
server_socket.listen(1)

print("Server listening...")

client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address}")

data = client_socket.recv(1024)
print("Received:", data.decode())

client_socket.sendall(b"Hello from server!")

client_socket.close()
server_socket.close()


"""
    
    
    
1- Server Code:

    Create Server Socket: The server socket is created using socket.socket() function, specifying the address family (AF_INET for IPv4) and socket type (SOCK_STREAM for TCP).

    Bind Server Socket: The server socket is bound to a specific IP address ('0.0.0.0', meaning it will accept connections from any available network interface) and port number (8080) using the bind() method.

    Listen for Connections: The server socket enters the listening state to wait for incoming connections. The listen() method is called with the parameter 1, indicating the maximum number of queued connections.

    Accept Connection: When a client attempts to connect, the accept() method is called on the server socket. This method blocks until a connection is established, at which point it returns a new socket object representing the connection and the address of the client.

    Receive Data: Once the connection is established, the server socket receives data from the client using the recv() method.

    Send Data: After receiving data, the server socket sends a response back to the client using the sendall() method.

    Close Sockets: Finally, both the client and server sockets are closed using the close() method to release the network resources.



2- Client Code:

    Create Client Socket: The client socket is created similarly to the server socket, specifying the address family and socket type.

    Connect to Server: The client socket attempts to connect to the server's IP address ('server_ip_address') and port number (8080) using the connect() method.

    Send Data: After the connection is established, the client socket sends data to the server using the sendall() method.

    Receive Data: The client socket then waits to receive a response from the server using the recv() method.

    Print Received Data: Once data is received, it is printed out to the console.

    Close Socket: Finally, the client socket is closed using the close() method.
        
    
    
    
    
    """