import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons, Button
from scipy.signal import butter, filtfilt, iirnotch, medfilt, savgol_filter
import pywt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import matplotlib.dates as mdates
import datetime
import os

def load_ecg_csv(filename):
    df = pd.read_csv(filename, header=None, names=["timestamp", "ecg"])
    timestamps = df["timestamp"].values
    ecg = df["ecg"].values
    return timestamps, ecg

def estimate_fs(timestamps):
    intervals = np.diff(timestamps)
    return 1.0 / np.mean(intervals)

def notch_filter(signal, fs, freq=50.0, Q=30.0):
    """
    Entfernt Netzbrummen.
    - fs: Abtastrate [Hz]
    - freq: Mittenfrequenz des Kerbfilters (50 Hz EU, 60 Hz US)
    - Q: Qualitätsfaktor (höher = schmalere Kerbe, weniger benachbarte Frequenzen werden gedämpft)
    """
    # Erzeuge IIR-Notch-Filterkoeffizienten (b=Zähler, a=Nenner) für Mittenfrequenz 'freq'
    b, a = iirnotch(w0=freq, Q=Q, fs=fs)

    # Zero-Phase-Filtering: vorwärts + rückwärts filtern, vermeidet Phasenverschiebung
    # Achtung: 'filtfilt' benötigt genügend Samples; bei sehr kurzen Signalen kann es instabil werden.
    return filtfilt(b, a, signal)


def bandpass_filter(signal, fs, low=0.5, high=40.0):
    """
    Bandpassfilter (Butterworth, 2. Ordnung): lässt Frequenzen zwischen 'low' und 'high' Hz passieren.
    """
    nyq = 0.5 * fs  # Nyquist-Frequenz
    # Normalisierte Grenzfrequenzen für Butterworth (müssen zwischen 0 und 1 liegen)
    low_n = low / nyq
    high_n = high / nyq
    # Normierter Grenzwert (zwischen 0 und 1)
    low_n = max(low_n, 1e-6)
    high_n = min(high_n, 0.999999)

    # Butterworth-Filter 2. Ordnung
    b, a = butter(N=2, Wn=[low_n, high_n], btype='band')
    # Zero-Phase-Filtering anwenden
    return filtfilt(b, a, signal)


def highpass_filter(signal, fs, cutoff=0.5):
    """
    Hochpass (Butterworth, 2. Ordnung): entfernt tieffrequente Anteile (z. B. Baseline-Drift).
    - cutoff: Grenzfrequenz in Hz
    Achtung: Zu hohe Grenzfrequenz kann ST-Segmente/R-Peaks verformen.
    """
    nyq = 0.5 * fs
    # Normierter Grenzwert (zwischen 0 und 1)
    wn = min(max(cutoff / nyq, 1e-6), 0.999999)
    b, a = butter(N=2, Wn=wn, btype='high')
    # Zero-Phase-Filtering anwenden
    return filtfilt(b, a, signal)


def lowpass_filter(signal, fs, cutoff=40.0):
    """
    Tiefpass (Butterworth, 2. Ordnung): entfernt hochfrequentes Rauschen.
    - cutoff: obere Grenzfrequenz in Hz
    Achtung: Zu niedrig gewählt -> dämpft steile Flanken (z. B. QRS-Komplexe).
    """
    nyq = 0.5 * fs
    # Normierter Grenzwert (zwischen 0 und 1), begrenzt zur Vermeidung numerischer Fehler
    wn = min(max(cutoff / nyq, 1e-6), 0.999999)
    b, a = butter(N=2, Wn=wn, btype='low')
    # Zero-Phase-Filtering anwenden
    return filtfilt(b, a, signal)


def median_filter(signal, kernel_size=5):
    """
    Median-Filter: robust gegen Ausreißer/Spikes (nichtlinear).
    - kernel_size: Fensterbreite (muss eine ungerade Zahl sein; bei geraden wählt SciPy intern eine passende Größe)
    Hinweis: Glättet, erhält aber Kanten besser als gleitender Mittelwert.
    """
    return medfilt(signal, kernel_size)


def wavelet_denoise(signal, wavelet='db6', level=3):
    """
    Wavelet-Denoising per Schwellenwert:
    - Weichschwellwert (soft threshold) jeden Koeffizientenvektor proportional zu dessen Standardabweichung
    - Rekonstruiert das Signal ohne starke Rauschanteile
    Parameter:
      - wavelet: z. B. 'db4', 'db6', 'sym8' (db6 oft gut für ECG)
      - level: Dekompositionslevel (abhängig von fs und Signalinhalt; zu hoch => Oversmoothing)
    Hinweis: Schwellenwert 0.5*std ist in der Praxis bewährt; je höher, desto stärker die Glättung.
    """
    # Diskrete Wavelet-Dekomposition -> Liste von Koeffizienten (cA_L, cD_L, ..., cD_1)
    coeffs = pywt.wavedec(signal, wavelet, level=level)

    # Soft-Thresholding pro Skala: dämpft kleine (rauschähnliche) Koeffizienten stärker
    thresholded = [
        pywt.threshold(c, value=np.std(c) * 0.5, mode='soft') for c in coeffs
    ]

    # Rekonstruktion; [:len(signal)] schützt vor Off-by-one bei Wavelet-Padding
    return pywt.waverec(thresholded, wavelet)[:len(signal)]


def savgol_filter_ecg(signal, window_length=11, polyorder=3):
    """
    Savitzky-Golay-Glättung:
    - Lokale Polynomanpassung über verschiebendes Fenster
    - Erhält Spitzen/Steigungen besser als einfacher Moving Average
    """
    if len(signal) < window_length:
        # Falls zu kurz: unverändert zurückgeben, statt Exception zu werfen
        return signal
    if window_length % 2 == 0:
        window_length += 1  # sicherstellen, dass ungerade
    polyorder = min(polyorder, window_length - 1)  # Konsistenz
    return savgol_filter(signal, window_length=window_length, polyorder=polyorder)


def moving_average(signal, window_size=5):
    """
    Gleitender Mittelwert:
    - Einfaches Tiefpass-Glättungsverfahren
    - 'same' belässt die Länge des Ausgangssignals
    Hinweis: Kanten werden geglättet, Peaks abgeflacht; keine Phasenkompensation (anders als filtfilt).
    """
    window = np.ones(window_size) / window_size
    return np.convolve(signal, window, mode='same')


def adaptive_threshold(signal, window_size=200):
    """
    Adaptive Baseline-/Trend-Entfernung mittels gleitendem Median:
    - Schätzt langsam driftende Basislinie im Zentrum eines Fensters
    - Subtrahiert diese Baseline
    Vorteile: Median ist robust gegen Ausreißer/Peaks
    """
    # Rolling-Median mit zentriertem Fenster; min_periods=1
    baseline = pd.Series(signal).rolling(
        window=window_size, min_periods=1, center=True
    ).median()

    return signal - baseline.values


def rms_envelope(signal, window_size=50):
    """
    RMS-Hüllkurve (Root Mean Square):
    - Nützlich z. B. zur Aktivitätserkennung oder für EKG-Amplitudenverläufe
    Schritte:
      1) Quadrat bilden
      2) Gleitenden Mittelwert (über die Quadrate)
      3) Quadratwurzel -> RMS
    """
    squared = np.square(signal)
    envelope = pd.Series(squared).rolling(
        window=window_size, min_periods=1, center=True
    ).mean()
    return np.sqrt(envelope).values

def apply_filters(ecg, fs,
                  use_notch=True, use_bandpass=True, use_median=True,
                  use_wavelet=True, use_savgol=False, use_movingavg=False,
                  use_highpass=False, use_lowpass=False,
                  use_adaptive=False, use_rms=False):
    if use_notch:
        ecg = notch_filter(ecg, fs)
    if use_bandpass:
        ecg = bandpass_filter(ecg, fs)
    if use_highpass:
        ecg = highpass_filter(ecg, fs)
    if use_lowpass:
        ecg = lowpass_filter(ecg, fs)
    if use_median:
        ecg = median_filter(ecg)
    if use_wavelet:
        ecg = wavelet_denoise(ecg)
    if use_savgol:
        ecg = savgol_filter_ecg(ecg)
    if use_movingavg:
        ecg = moving_average(ecg)
    if use_adaptive:
        ecg = adaptive_threshold(ecg)
    if use_rms:
        ecg = rms_envelope(ecg)
    return ecg

def main():
    Tk().withdraw()
    filename = askopenfilename(title="Choose your ECG file",
                               filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
    if not filename:
        print("No file selected.")
        return

    timestamps, ecg = load_ecg_csv(filename)
    fs = estimate_fs(timestamps)
    print(f"Loaded: {filename}")
    print(f"Estimated sampling rate: {fs:.2f} Hz")

    datetime_array = [datetime.datetime.fromtimestamp(ts) for ts in timestamps]
    mpl_times = mdates.date2num(datetime_array)

    use_notch = [True]
    use_bandpass = [True]
    use_median = [True]
    use_wavelet = [True]
    use_savgol = [False]
    use_movingavg = [False]
    use_highpass = [False]
    use_lowpass = [False]
    use_adaptive = [False]
    use_rms = [False]

    filtered = [apply_filters(ecg, fs,
        use_notch[0], use_bandpass[0], use_median[0], use_wavelet[0],
        use_savgol[0], use_movingavg[0], use_highpass[0], use_lowpass[0],
        use_adaptive[0], use_rms[0])]

    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.60, top=0.85)
    fig.suptitle(f"ECG Recording Date: {datetime_array[0].strftime('%Y-%m-%d')}", fontsize=14)
    ax.set_xlabel("Time (HH:MM:SS)")
    ax.set_ylabel("ECG")
    ax.set_title("ECG Signal Viewer with Real Timestamps")
    ax.grid(True)
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))

    l_raw, = ax.plot([], [], label="Original", color='lightgray', linewidth=1.5)
    l_filt, = ax.plot([], [], label="Filtered", color='blue', linewidth=2)
    ax.legend()

    ax_slider = plt.axes([0.15, 0.45, 0.7, 0.03])
    duration = (mpl_times[-1] - mpl_times[0]) * 24 * 3600
    if duration < 10:
        duration = 10
    slider = Slider(ax_slider, 'Scroll', 0, duration - 10, valinit=0, valstep=0.1)

    ax_vis_check = plt.axes([0.15, 0.30, 0.2, 0.1])
    vis_check = CheckButtons(ax_vis_check, ['Original', 'Filtered'], [True, True])
    visible_raw = [True]
    visible_filt = [True]

    ax_filter_check = plt.axes([0.45, 0.05, 0.45, 0.35])
    filter_check = CheckButtons(ax_filter_check,
        ['Notch', 'Bandpass', 'Median', 'Wavelet', 'Savitzky-Golay', 'Moving Avg',
         'High-pass', 'Low-pass', 'Adaptive Thresh', 'RMS Envelope'],
        [use_notch[0], use_bandpass[0], use_median[0], use_wavelet[0],
         use_savgol[0], use_movingavg[0], use_highpass[0], use_lowpass[0],
         use_adaptive[0], use_rms[0]])

    ax_save_btn = plt.axes([0.15, 0.18, 0.2, 0.05])
    btn_save = Button(ax_save_btn, "Save Filtered Data")

    def refresh_filtered():
        return apply_filters(ecg, fs,
            use_notch[0], use_bandpass[0], use_median[0], use_wavelet[0],
            use_savgol[0], use_movingavg[0], use_highpass[0], use_lowpass[0],
            use_adaptive[0], use_rms[0])

    def update_plot(val):
        start_sec = slider.val
        end_sec = start_sec + 10
        start_time = mpl_times[0] + start_sec / (24 * 3600)
        end_time = mpl_times[0] + end_sec / (24 * 3600)
        idx = np.where((mpl_times >= start_time) & (mpl_times <= end_time))

        l_raw.set_data(mpl_times[idx[0]], ecg[idx[0]])
        l_filt.set_data(mpl_times[idx[0]], filtered[0][idx[0]])
        ax.set_xlim(start_time, end_time)

        ymin = min(np.min(ecg[idx[0]]), np.min(filtered[0][idx[0]])) - 10
        ymax = max(np.max(ecg[idx[0]]), np.max(filtered[0][idx[0]])) + 10
        ax.set_ylim(ymin, ymax)

        l_raw.set_visible(visible_raw[0])
        l_filt.set_visible(visible_filt[0])
        fig.canvas.draw_idle()

    def toggle_visibility(label):
        if label == 'Original':
            visible_raw[0] = not visible_raw[0]
        elif label == 'Filtered':
            visible_filt[0] = not visible_filt[0]
        update_plot(slider.val)

    def save_filtered_data(timestamps, filtered_signal, original_filename):
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()

        base_name = os.path.basename(original_filename)
        name_without_ext = os.path.splitext(base_name)[0]
        default_name = name_without_ext + "_Filtered.csv"

        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv")],
            title="Save Filtered ECG Data As"
        )

        if file_path:
            df_out = pd.DataFrame({
                'timestamp': timestamps,
                'ecg': filtered_signal
            })
            df_out.to_csv(file_path, index=False, header=False)
            print(f"✅ Filtered data saved to: {file_path}")
        else:
            print("❌ Save cancelled.")

    def save_filtered_button_clicked(event):
        save_filtered_data(timestamps, filtered[0], filename)

    def toggle_filter(label):
        if label == 'Notch': use_notch[0] = not use_notch[0]
        elif label == 'Bandpass': use_bandpass[0] = not use_bandpass[0]
        elif label == 'Median': use_median[0] = not use_median[0]
        elif label == 'Wavelet': use_wavelet[0] = not use_wavelet[0]
        elif label == 'Savitzky-Golay': use_savgol[0] = not use_savgol[0]
        elif label == 'Moving Avg': use_movingavg[0] = not use_movingavg[0]
        elif label == 'High-pass': use_highpass[0] = not use_highpass[0]
        elif label == 'Low-pass': use_lowpass[0] = not use_lowpass[0]
        elif label == 'Adaptive Thresh': use_adaptive[0] = not use_adaptive[0]
        elif label == 'RMS Envelope': use_rms[0] = not use_rms[0]

        filtered[0] = refresh_filtered()
        update_plot(slider.val)

    slider.on_changed(update_plot)
    vis_check.on_clicked(toggle_visibility)
    filter_check.on_clicked(toggle_filter)
    btn_save.on_clicked(save_filtered_button_clicked)

    update_plot(0)
    plt.show()

if __name__ == "__main__":
    main()
