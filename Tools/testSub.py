
import zmq
from utils import getmylogger


log = getmylogger(__name__)


       
if __name__ == "__main__":
    ctx  = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("ipc://SHARED")
    sub.setsockopt( zmq.SUBSCRIBE, b"")
    log.info("Begin Test Sub")
    try:
        while True:
            try:
                data = sub.recv().decode()
                print(data)
            except Exception as e:
                log.error(f"Expeption in testSub: {e}")
                break
        log.info("Exit test SUB")
        sub.close()
    except KeyboardInterrupt:
        print("Exit text sub ")

