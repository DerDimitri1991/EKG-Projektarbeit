import json
import threading
import numpy as np
import paho.mqtt.client as mqtt
import neurokit2 as nk
import warnings
import time


warnings.filterwarnings("ignore")

MQTT_BROKER = "192.168.178.40"
MQTT_PORT = 1883
CHUNK_SIZE = 5
NUM_CHUNKS = 2500
BUFFER_SIZE = CHUNK_SIZE * NUM_CHUNKS
MAX_RATES = 20
AGE = 34  # Adjust per athlete if needed for Zone warmup etc.
NUM_DEVICES = 10  # Change this to set the number of ECG devices


class ECGDeviceHandler:
    def __init__(self, device_id):
        self.device_id = device_id
        self.sub_topic = f"ecg/device{device_id}"
        self.pub_topic = f"processed/device{device_id}"
        self.buffer_ecg = np.zeros(BUFFER_SIZE)
        self.buffer_ts = np.zeros(BUFFER_SIZE)
        self.recent_rates = []
        self.processing = False
        self.lock = threading.Lock()

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        connected = False
        while not connected:
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
                connected = True
            except Exception as e:
                print(f"[Device {self.device_id}] MQTT connection failed: {e}")
                print("Retrying in 5 seconds...")
                time.sleep(5)


        threading.Thread(target=self.client.loop_forever, daemon=True).start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[Device {self.device_id}] Connected. Subscribing to {self.sub_topic}")
            client.subscribe(self.sub_topic)
        else:
            print(f"[Device {self.device_id}] Connection failed")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode().strip())
            ecg = np.array(payload.get("ecg", []))
            ts = np.array(payload.get("ts", []))

            if len(ts) > 1:
                interval = np.mean(np.diff(ts))
                rate = 1_000_000 / interval
                if np.isfinite(rate) and rate > 0:
                    self.recent_rates.append(rate)
                    if len(self.recent_rates) > MAX_RATES:
                        self.recent_rates.pop(0)

            with self.lock:
                self.buffer_ecg = np.concatenate((self.buffer_ecg[len(ecg):], ecg))
                self.buffer_ts = np.concatenate((self.buffer_ts[len(ts):], ts))

            if len(self.buffer_ecg) >= BUFFER_SIZE and len(self.recent_rates) >= 5 and not self.processing:
                avg_rate = np.mean(self.recent_rates)
                threading.Thread(
                    target=self.process_ecg,
                    args=(self.buffer_ecg.copy(), avg_rate),
                    daemon=True
                ).start()

        except Exception as e:
            print(f"[Device {self.device_id}] Message error:", e)

    def process_ecg(self, signal, rate):
        self.processing = True
        try:
            if np.isnan(signal).any() or np.std(signal) < 0.001:
                print(f"[Device {self.device_id}] Skipped invalid signal")
                return

            signals, info = nk.ecg_process(signal, sampling_rate=rate)
            hrv = nk.hrv(signals, sampling_rate=rate, show=False)
            bpm_mean = float(signals["ECG_Rate"].mean())
            rmssd = hrv.get("HRV_RMSSD", [np.nan]).iloc[0]
            zone_ratio = bpm_mean / (220 - AGE)

            if zone_ratio < 0.6:
                zone = "Warm-up"
            elif zone_ratio < 0.7:
                zone = "Fat burn"
            elif zone_ratio < 0.8:
                zone = "Endurance"
            elif zone_ratio < 0.9:
                zone = "Hard"
            else:
                zone = "Maximum"

            ecg_quality = float(np.nanmean(nk.ecg_quality(signal, sampling_rate=rate)))
            ecg_rsp = float(np.nanmean(nk.ecg_rsp(signal, sampling_rate=rate)))

            result = {
                "device": self.device_id,
                "bpm_mean": round(bpm_mean, 2),
                "sampling_rate": round(rate, 2),
                "hrv_rmssd": round(rmssd, 2) if np.isfinite(rmssd) else None,
                "ecg_quality": round(ecg_quality, 3),
                "ecg_rsp_mean": round(ecg_rsp, 3),
                "zone": zone
            }

            self.client.publish(self.pub_topic, json.dumps(result))
            print(f"[Device {self.device_id}] Published result to {self.pub_topic}")

        except Exception as e:
            print(f"[Device {self.device_id}] Processing error:", e)

        finally:
            self.processing = False


# === Startup ===
if __name__ == "__main__":
    handlers = [ECGDeviceHandler(device_id=i + 1) for i in range(NUM_DEVICES)]

    print(f"\nStarted {NUM_DEVICES} device handler(s). Press Ctrl+C to exit.")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Shutting down.")
