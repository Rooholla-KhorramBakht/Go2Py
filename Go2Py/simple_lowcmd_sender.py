# from unitree_go.msg.dds_ import LowState_, LowCmd_, MotorCmd_, BmsCmd_
from msgs import LowCmd
import cyclonedds.idl.types as types
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.topic import Topic
from cyclonedds.util import duration
import time


# Create a DomainParticipant, your entrypoint to DDS
# Created in the default domain
dp = DomainParticipant(0)
# Create a Topic with topic name "Hello" and as datatype "HelloWorld" structs.
tp = Topic(dp, "go2py/lowcmd", LowCmd)
dw = DataWriter(dp, tp)
cmd = LowCmd(
    q=12 * [0.],
    dq=12 * [0.],
    tau_ff=12 * [0.],
    kp=12 * [0.],
    kv=12 * [0.],
    e_stop=0
)
while True:
    dw.write(cmd)
    time.sleep(0.01)
