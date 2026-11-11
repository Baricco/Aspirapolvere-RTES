#!/usr/bin/env python3
"""Porta HTTPS davanti al backend di sviluppo, come fa Apache in produzione.

Il backend sul PC ascolta in HTTP (comodo per l'app web nel browser); il programma del robot accetta solo HTTPS. Questo
inoltro accetta connessioni TLS con il certificato autofirmato di dev/make-cert.sh e passa i byte al backend.
Uso: tls_proxy.py --cert localhost-cert.pem --key localhost-key.pem [--listen 5443] [--backend 5000]
"""
import argparse
import asyncio
import ssl

DEFAULT_LISTEN_PORT = 5443
DEFAULT_BACKEND_PORT = 5000
HOST = "127.0.0.1"
BUFFER_BYTES = 65536


async def copy(reader, writer):
    """Passa i byte da una parte all'altra finché una delle due chiude."""
    try:
        while data := await reader.read(BUFFER_BYTES):
            writer.write(data)
            await writer.drain()
    except (ConnectionError, ssl.SSLError):
        pass
    finally:
        writer.close()


async def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cert", required=True)
    parser.add_argument("--key", required=True)
    parser.add_argument("--listen", type=int, default=DEFAULT_LISTEN_PORT)
    parser.add_argument("--backend", type=int, default=DEFAULT_BACKEND_PORT)
    arguments = parser.parse_args()

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(arguments.cert, arguments.key)

    async def handle(client_reader, client_writer):
        try:
            backend_reader, backend_writer = await asyncio.open_connection(HOST, arguments.backend)
        except OSError:
            client_writer.close()
            return
        await asyncio.gather(copy(client_reader, backend_writer), copy(backend_reader, client_writer))

    server = await asyncio.start_server(handle, HOST, arguments.listen, ssl=context)
    print(f"https://localhost:{arguments.listen} -> http://{HOST}:{arguments.backend}", flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
