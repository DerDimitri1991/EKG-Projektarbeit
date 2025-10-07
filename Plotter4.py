import pandas as pd
import neurokit2 as nk
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons, Button
import zoneinfo
import tkinter as tk
from tkinter import filedialog

# === SETTINGS ===
label_max_length = 30

# === LOAD DATA ===
# Use Tkinter to open a file selection dialog
root = tk.Tk()
root.withdraw()  # Hide the main window
file_path = filedialog.askopenfilename(
    title="Open ECG CSV File",
    filetypes=[("CSV files", "*.csv")]
)

if not file_path:
    raise ValueError("No file selected. Exiting.")

df = pd.read_csv(file_path, header=None, names=["utc", "ecg"])

df["time"] = pd.to_datetime(df["utc"], unit="s", utc=True).dt.tz_convert("Europe/Berlin")
df.set_index("time", inplace=True)

# === SAMPLING RATE ===
time_deltas = df.index.to_series().diff().dt.total_seconds().dropna()
valid_deltas = time_deltas[time_deltas > 0]
if valid_deltas.empty:
    raise ValueError("Keine gültigen Zeitdifferenzen gefunden! Die Timestamps sind wahrscheinlich identisch.")
sampling_rate = round(1 / valid_deltas.median())
print(f" Estimated sampling rate: {sampling_rate} Hz")

# === ECG PROCESSING ===
signals, info = nk.ecg_process(df["ecg"], sampling_rate=sampling_rate)
time = df.index.to_numpy()

# === SIGNALS ===
all_signals = signals.columns.tolist()
default_signals = ["ECG_Clean"]

# === ZOOM FUNCTION ===
def zoom_factory(ax, base_scale=1.2):
    def zoom(event):
        if event.inaxes != ax:
            return
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        xdata = event.xdata
        ydata = event.ydata

        if event.button == 'up':
            scale_factor = 1 / base_scale
        elif event.button == 'down':
            scale_factor = base_scale
        else:
            return

        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor

        relx = (xdata - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0])
        rely = (ydata - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0])

        ax.set_xlim([xdata - new_width * relx, xdata + new_width * (1 - relx)])
        ax.set_ylim([ydata - new_height * rely, ydata + new_height * (1 - rely)])
        ax.figure.canvas.draw_idle()
    ax.figure.canvas.mpl_connect('scroll_event', zoom)

# === LABELS ===
def shorten_label(name, max_len):
    if name.startswith("ECG_"):
        name = name[4:]
    return name.replace("_", " ")[:max_len]

display_labels = [shorten_label(name, label_max_length) for name in all_signals]
label_map = dict(zip(display_labels, all_signals))

# === FIGURE ===
fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(left=0.3, bottom=0.3, top=0.92)

hrv_text = fig.text(0.01, 0.95, f"Sampling rate: {sampling_rate} Hz", fontsize=10, color="blue")
fig.text(0.02, 0.92, "ECG", fontsize=12, weight="bold")
date_display = fig.text(0.75, 0.95, "", fontsize=10, color="darkgreen")

active_signals = default_signals[:]

lines = {}
for sig in active_signals:
    lines[sig], = ax.plot(time, signals[sig], label=sig)
ax.set_title("Overlapping Signals", fontsize=10)
ax.legend(fontsize=8)
ax.grid(True)
zoom_factory(ax)

# === CHECKBOX ===
ax_check = plt.axes([0.01, 0.3, 0.26, 0.6])
visibility = [sig in active_signals for sig in all_signals]
check = CheckButtons(ax_check, display_labels, visibility)
for text in check.labels:
    text.set_fontsize(9)

# === SCROLL SLIDER ===
window_size = 5
duration = (df.index[-1] - df.index[0]).total_seconds()
ax_slider = plt.axes([0.3, 0.23, 0.6, 0.03])
slider_scroll = Slider(ax_slider, "Scroll", 0, duration - window_size, valinit=0)
slider_scroll.valtext.set_visible(False)

def update_scroll(val):
    start = df.index[0] + pd.Timedelta(seconds=val)
    end = start + pd.Timedelta(seconds=window_size)
    ax.set_xlim(start, end)
    date_display.set_text(start.strftime('%d-%m-%Y %H:%M:%S %Z'))
    fig.canvas.draw_idle()

slider_scroll.on_changed(update_scroll)

# === START & END SLIDERS ===
ax_start = plt.axes([0.3, 0.17, 0.6, 0.03])
slider_start = Slider(ax_start, "Start Selection", 0, duration, valinit=0)
slider_start.valtext.set_visible(False)

ax_end = plt.axes([0.3, 0.09, 0.6, 0.03])
slider_end = Slider(ax_end, "End Selection", 0, duration, valinit=1)
slider_end.valtext.set_visible(False)

text_start = fig.text(0.6, 0.14, "", ha="center", fontsize=9)
text_end = fig.text(0.6, 0.06, "", ha="center", fontsize=9)

def update_slider_labels():
    start_time = df.index[0] + pd.Timedelta(seconds=slider_start.val)
    end_time = df.index[0] + pd.Timedelta(seconds=slider_end.val)
    text_start.set_text(start_time.strftime('%d-%m-%Y %H:%M:%S'))
    text_end.set_text(end_time.strftime('%d-%m-%Y %H:%M:%S'))
    fig.canvas.draw_idle()

slider_start.on_changed(lambda val: update_slider_labels())
slider_end.on_changed(lambda val: update_slider_labels())
update_slider_labels()

# === SAVE BUTTON ===
ax_button = plt.axes([0.38, 0.005, 0.2, 0.05])
btn_save = Button(ax_button, "Save Selection")

def save_selection(event):
    start_sec = slider_start.val
    end_sec = slider_end.val
    if start_sec >= end_sec:
        print(" Start must be before end.")
        return
    start_time = df.index[0] + pd.Timedelta(seconds=start_sec)
    end_time = df.index[0] + pd.Timedelta(seconds=end_sec)
    selection = df.loc[start_time:end_time]

    if selection.empty:
        print(" No data in range.")
        return

    export_df = selection[["utc", "ecg"]]
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.asksaveasfilename(
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv")],
        title="Save ECG Selection As"
    )

    if file_path:
        export_df.to_csv(file_path, index=False, header=False)
        print(f" Saved: {start_time} to {end_time} → {file_path}")
    else:
        print(" Save cancelled.")

btn_save.on_clicked(save_selection)

# === HRV BUTTON ===
ax_hrv = plt.axes([0.6, 0.005, 0.2, 0.05])
btn_hrv = Button(ax_hrv, "Calculate HRV")

def calculate_hrv(event):
    start_sec = slider_start.val
    end_sec = slider_end.val
    if start_sec >= end_sec:
        hrv_text.set_text(" Invalid time range")
        fig.canvas.draw_idle()
        return

    start_time = df.index[0] + pd.Timedelta(seconds=start_sec)
    end_time = df.index[0] + pd.Timedelta(seconds=end_sec)
    selection = df.loc[start_time:end_time]

    if selection.empty:
        hrv_text.set_text(" No data in range")
        fig.canvas.draw_idle()
        return

    try:
        _, info = nk.ecg_peaks(selection["ecg"].values, sampling_rate=sampling_rate)
        hrv_metrics = nk.hrv_time(info, sampling_rate=sampling_rate, show=False)
        rmssd = hrv_metrics["HRV_RMSSD"].values[0]
        sdnn = hrv_metrics["HRV_SDNN"].values[0]
        rpeaks = info["ECG_R_Peaks"]
        mean_hr = nk.ecg_rate(rpeaks, sampling_rate=sampling_rate).mean()
        hrv_text.set_text(f"Sampling rate: {sampling_rate} Hz | RMSSD: {rmssd:.1f} ms | SDNN: {sdnn:.1f} ms | BPM: {mean_hr:.1f}")
    except Exception as e:
        hrv_text.set_text(" HRV calc failed")
        print(f" Error: {e}")

    fig.canvas.draw_idle()

btn_hrv.on_clicked(calculate_hrv)

# === ANALYZE BUTTON ===
ax_analyze = plt.axes([0.16, 0.005, 0.2, 0.05])
btn_analyze = Button(ax_analyze, "Analyze Selection")

def analyze_selection(event):
    start_sec = slider_start.val
    end_sec = slider_end.val
    if start_sec >= end_sec:
        print(" Invalid time range")
        return

    start_time = df.index[0] + pd.Timedelta(seconds=start_sec)
    end_time = df.index[0] + pd.Timedelta(seconds=end_sec)
    selection = df.loc[start_time:end_time]

    if selection.empty:
        print(" No data in range")
        return

    try:
        signals, info = nk.ecg_process(selection["ecg"].values, sampling_rate=sampling_rate)
        nk.ecg_plot(signals, info)
        plt.show()
    except Exception as e:
        print(f" ECG processing failed: {e}")

btn_analyze.on_clicked(analyze_selection)

# === RESET ZOOM BUTTON ===
ax_reset = plt.axes([0.82, 0.005, 0.15, 0.05])
btn_reset = Button(ax_reset, "Reset Zoom")

def reset_zoom(event):
    ax.set_xlim(df.index[0], df.index[-1])
    ax.set_ylim(signals.min().min(), signals.max().max())
    fig.canvas.draw_idle()

btn_reset.on_clicked(reset_zoom)

# === CHECKBOX CALLBACK ===
def update_visibility(label):
    global active_signals
    if label in label_map:
        name = label_map[label]
        if name in active_signals:
            active_signals.remove(name)
        else:
            active_signals.append(name)

    ax.clear()
    ecg_clean = signals["ECG_Clean"]
    max_amp = ecg_clean.max()
    offset = ecg_clean.median()

    binary_signals = {
        "ECG_Peaks", "ECG_R_Peaks", "ECG_PhaseAtrial", "ECG_PhaseVentricular",
        "ECG_CompletionAtrial", "ECG_CompletionVentricular", "ECG_Quality"
    }

    for sig in active_signals:
        y = signals[sig]
        if sig in binary_signals or set(y.unique()) <= {0, 1, -1}:
            y = y * max_amp * 1.2 + offset
        ax.plot(time, y, label=sig)

    ax.set_title("Overlapping Signals", fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(True)
    update_scroll(slider_scroll.val)

check.on_clicked(update_visibility)

# === INIT ===
update_scroll(0)
plt.show()
input("Press ENTER to exit...")