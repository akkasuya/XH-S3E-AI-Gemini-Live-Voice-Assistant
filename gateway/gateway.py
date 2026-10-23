"""Local gateway between the ESP32-S3 (raw PCM over WebSocket) and Gemini Live.

ESP32  --binary 16 kHz PCM-->  gateway  --Live API audio-->  Gemini
ESP32  <--binary 24 kHz PCM--  gateway  <--Live API audio--  Gemini

Run:
    export GEMINI_API_KEY=...            # never hard-code the key
    pip install fastapi uvicorn google-genai
    python gateway.py
"""

import asyncio
import os

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from google import genai
from google.genai import types

GEMINI_API_KEY = os.environ["GEMINI_API_KEY"]
GEMINI_MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.1-flash-live-preview")
GEMINI_VOICE = os.environ.get("GEMINI_VOICE", "Vega")
HOST = os.environ.get("GATEWAY_HOST", "0.0.0.0")
PORT = int(os.environ.get("GATEWAY_PORT", "8008"))

client = genai.Client(api_key=GEMINI_API_KEY)

LIVE_CONFIG = types.LiveConnectConfig(
    response_modalities=["AUDIO"],
    speech_config=types.SpeechConfig(
        voice_config=types.VoiceConfig(
            prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=GEMINI_VOICE)
        )
    ),
)

app = FastAPI()


async def esp32_to_gemini(esp32_ws: WebSocket, session) -> None:
    """Forward binary mic PCM from the ESP32 to Gemini."""
    while True:
        message = await esp32_ws.receive()
        if message["type"] == "websocket.disconnect":
            return
        data = message.get("bytes")
        if data:
            await session.send_realtime_input(
                audio=types.Blob(data=data, mime_type="audio/pcm;rate=16000")
            )


async def gemini_to_esp32(esp32_ws: WebSocket, session) -> None:
    """Forward Gemini's audio replies to the ESP32 as raw binary PCM."""
    async for response in session.receive():
        content = response.server_content
        if content is None or content.model_turn is None:
            continue
        for part in content.model_turn.parts:
            if part.inline_data and part.inline_data.data:
                await esp32_ws.send_bytes(part.inline_data.data)


@app.websocket("/media-stream")
async def media_stream(esp32_ws: WebSocket):
    await esp32_ws.accept()
    print("ESP32-S3 connected to local gateway")

    try:
        async with client.aio.live.connect(model=GEMINI_MODEL, config=LIVE_CONFIG) as session:
            print(f"Gemini Live session open ({GEMINI_MODEL}, voice={GEMINI_VOICE})")
            tasks = [
                asyncio.create_task(esp32_to_gemini(esp32_ws, session)),
                asyncio.create_task(gemini_to_esp32(esp32_ws, session)),
            ]
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            for task in done:
                exc = task.exception()
                if exc is not None:
                    print(f"Stream error: {exc!r}")
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        print(f"Gemini session error: {exc!r}")
    finally:
        print("ESP32-S3 disconnected")
        try:
            await esp32_ws.close()
        except RuntimeError:
            pass


if __name__ == "__main__":
    uvicorn.run(app, host=HOST, port=PORT)
