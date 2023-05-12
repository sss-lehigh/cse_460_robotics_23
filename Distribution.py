import xmlrpc.server
import threading
import xmlrpc.client
import os

class DistributionFuncs:

    def __init__(self):
        pass

    def heartbeat(self):
        return True

class DistributionServer(threading.Thread):
    def __init__(self, port):
        super().__init__()
        self.server = xmlrpc.server.SimpleXMLRPCServer(("0.0.0.0", port))
        self.server.register_instance(DistributionFuncs())
        self.start()

    def run(self):
        self.server.serve_forever()

class Distribution:
    def __init__(self, server, remote_addrs_dict):
        self.server = server
        self.clients = dict()
        print(remote_addrs_dict)
        for name, addr in remote_addrs_dict.items():
            self.clients[name] = xmlrpc.client.ServerProxy(addr)

    def call(self, server, call_name, *args):
        fn = getattr(self.clients[server],call_name)
        return fn(*args)

if __name__ == "__main__":
    server = Distribution(DistributionServer(8080), dict())
    servers = dict()
    servers[0] = "http://localhost:8080"
    clients = Distribution(None, servers)
    print(clients.call(0, "heartbeat"))

    os._exit(0)
