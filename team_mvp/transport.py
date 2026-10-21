"""Datagram port and its production UDP adapter. No device policy lives here."""
from abc import ABC, abstractmethod
import socket


class DatagramTransport(ABC):
    @property
    @abstractmethod
    def address(self) -> tuple[str,int]: ...
    @abstractmethod
    def sendto(self, packet: bytes, address: tuple[str,int]) -> None: ...
    @abstractmethod
    def recvfrom(self, size: int) -> tuple[bytes,tuple[str,int]]: ...
    @abstractmethod
    def close(self) -> None: ...


class UdpTransport(DatagramTransport):
    def __init__(self,bind="127.0.0.1",port=5000):
        self._socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            self._socket.bind((bind,port))
            self._socket.settimeout(.02)
        except Exception:
            self._socket.close()
            raise
    @property
    def address(self): return self._socket.getsockname()
    def sendto(self,packet,address): self._socket.sendto(packet,address)
    def recvfrom(self,size): return self._socket.recvfrom(size)
    def close(self): self._socket.close()
