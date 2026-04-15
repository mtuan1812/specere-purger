
from app_state import RuntimeState
from web_server import create_server

def main():
    runtime = RuntimeState()
    server, poller = create_server(runtime)
    poller.start()
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
