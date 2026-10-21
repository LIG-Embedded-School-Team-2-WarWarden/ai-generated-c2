"""Explicit OBS workflow states; illegal shortcuts cannot resume collection."""
from dataclasses import dataclass,asdict
from enum import Enum


class ObserverPhase(str,Enum):
    WAIT_EFF_STOP="WAIT_EFF_STOP"
    WAIT_COLLECTION_STOP="WAIT_COLLECTION_STOP"
    OPENING="OPENING"
    MOVING="MOVING"
    STOPPING="STOPPING"
    COLLECTING="COLLECTING"
    COMPLETED="COMPLETED"
    FAILED="FAILED"


_ALLOWED={
    ObserverPhase.WAIT_EFF_STOP:{ObserverPhase.WAIT_COLLECTION_STOP,ObserverPhase.FAILED},
    ObserverPhase.WAIT_COLLECTION_STOP:{ObserverPhase.OPENING,ObserverPhase.COMPLETED,ObserverPhase.FAILED},
    ObserverPhase.OPENING:{ObserverPhase.MOVING,ObserverPhase.FAILED},
    ObserverPhase.MOVING:{ObserverPhase.COLLECTING,ObserverPhase.STOPPING,ObserverPhase.FAILED},
    ObserverPhase.STOPPING:{ObserverPhase.WAIT_COLLECTION_STOP,ObserverPhase.FAILED},
    ObserverPhase.COLLECTING:{ObserverPhase.COMPLETED,ObserverPhase.FAILED},
    ObserverPhase.COMPLETED:set(),ObserverPhase.FAILED:set(),
}


@dataclass
class ObserverOperation:
    phase:ObserverPhase
    session:str
    started:float
    pan:float=0
    tilt:float=0
    reason:str=""
    operation:str="POINT"
    sample:int=0
    ack:bool=False
    failed:bool=False

    def transition(self,phase:ObserverPhase):
        if phase==self.phase:
            return
        if phase not in _ALLOWED[self.phase]:
            raise ValueError(f"illegal OBS transition: {self.phase.value} -> {phase.value}")
        self.phase=phase

    @property
    def terminal(self): return self.phase in (ObserverPhase.COMPLETED,ObserverPhase.FAILED)

    def snapshot(self):
        value=asdict(self)
        value["state"]=value.pop("phase").value
        return value
