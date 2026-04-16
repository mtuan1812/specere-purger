import signal
import sys
from app_state import RuntimeState
from web_server import create_server

def main():
    runtime = RuntimeState()
    server, poller = create_server(runtime)
    poller.start()

    # Catch SIGTERM from kill/pkill and turn it into a KeyboardInterrupt
    def handle_sigterm(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, handle_sigterm)

    print("Serving on http://0.0.0.0:8000")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        try: server.shutdown()
        except Exception: pass
        try: server.server_close()
        except Exception: pass
        runtime.shutdown()

if __name__ == "__main__":
    main()
