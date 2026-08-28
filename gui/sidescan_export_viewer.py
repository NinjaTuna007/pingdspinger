#!/usr/bin/env python3
"""Offline sidescan export viewer — async load, live-GUI-identical viz knobs.

Navigation: wheel zoom · drag pan · double-click / Fit / 0 = fit · +/- zoom

Viz pipeline matches ``gui/sonar_control_gui.py::ss_render_bgr`` (nadir →
flatten → log window → CLAHE → despeckle → colormap). Speed-comp / stretch
are extra and do not drop along-track coverage.
"""

from __future__ import annotations

import json
import sys
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from tkinter import filedialog, ttk
import tkinter as tk

import numpy as np

_HERE = Path(__file__).resolve().parent
# Share zip: ./pingdsp_driver/sidescan_image.py next to this file.
# Repo: ../pingdsp_driver/pingdsp_driver/sidescan_image.py
for _root in (_HERE, _HERE.parent / "pingdsp_driver"):
    if (_root / "pingdsp_driver" / "sidescan_image.py").is_file():
        sys.path.insert(0, str(_root))
        break

from pingdsp_driver import sidescan_image as ssi  # noqa: E402

try:
    import cv2
    from PIL import Image, ImageTk
except ImportError as e:
    raise SystemExit(f"Need opencv-python + Pillow: {e}") from e


def _first_existing(*candidates: Path) -> Path | None:
    for p in candidates:
        if p.is_dir() or p.is_file():
            return p
    return None


# Prefer bundled share-zip layout, then in-repo defaults.
DEFAULT_DIR = _first_existing(
    _HERE / "data",
    _HERE.parent / "data",
    Path(
        "/media/shekharu/DATA/OneDrive/SMaRC PhD/Datasets/orebro/sidescan_fullres"
    ),
) or _HERE

_DEFAULT_JSON = _first_existing(
    _HERE / "config" / "sidescan_default.json",
    _HERE / "sidescan" / "config" / "sidescan_default.json",
) or (_HERE / "sidescan" / "config" / "sidescan_default.json")

# Soft-cap only insane speed-comp expansions (Orebro max raw ~32k rows).
_MAX_COMPOSE_H = 100_000
_ZOOM_MIN = 0.02
_ZOOM_MAX = 32.0
_CLAHE_GRID = 8  # same as live GUI ss_clahe_grid

# Fallback = live SS_DEFAULT_PRESET / sidescan_default.json
_VIZ_DEFAULTS = {
    "log_min": 11.5,
    "log_max": 15.0,
    "gamma": 1.0,
    "nadir_bins": 0,
    "flatten": 0.70,
    "clahe": 0.5,
    "despeckle": 0,
    "colormap": "bronze",
}


def _load_viz_defaults() -> dict:
    d = dict(_VIZ_DEFAULTS)
    try:
        with open(_DEFAULT_JSON, encoding="utf-8") as f:
            disk = json.load(f)
        for k in _VIZ_DEFAULTS:
            if k in disk:
                d[k] = disk[k]
    except (OSError, json.JSONDecodeError, TypeError):
        pass
    return d


def speed_resample(
    log_img: np.ndarray,
    along_m: np.ndarray,
    range_res_m: float,
    stretch: float = 1.0,
    max_rows: int = _MAX_COMPOSE_H,
) -> np.ndarray:
    """Resample so 1 px ≈ range_res/stretch metres; covers full along_m span."""
    if log_img.ndim != 2 or along_m.size != log_img.shape[0]:
        return log_img
    if along_m.size < 2 or range_res_m <= 0:
        return log_img
    s = np.asarray(along_m, dtype=np.float64)
    s = s - s[0]
    total = float(s[-1])
    if total <= 1e-3:
        return np.asarray(log_img, dtype=np.float32)
    dx = float(range_res_m) / max(float(stretch), 1e-3)
    n_out = max(int(np.ceil(total / dx)) + 1, 2)
    n_out = min(n_out, int(max_rows))
    s_out = np.linspace(0.0, total, n_out)  # full track, possibly coarser dx
    idx = np.interp(s_out, s, np.arange(log_img.shape[0], dtype=np.float64))
    i0 = np.clip(np.floor(idx).astype(np.int32), 0, log_img.shape[0] - 1)
    i1 = np.clip(i0 + 1, 0, log_img.shape[0] - 1)
    a = (idx - i0).astype(np.float32)[:, None]
    src = np.asarray(log_img)
    r0 = src[i0].astype(np.float32, copy=False)
    r1 = src[i1].astype(np.float32, copy=False)
    return r0 * (1.0 - a) + r1 * a


def integrate_along_track(stamps: np.ndarray, speeds: np.ndarray) -> np.ndarray:
    n = stamps.size
    out = np.zeros(n, dtype=np.float64)
    if n < 2:
        return out
    dt = np.clip(np.diff(stamps), 0.0, 2.0)
    out[1:] = np.cumsum(np.maximum(speeds[:-1], 0.0) * dt)
    return out


def load_export(path: Path, progress=None) -> dict:
    path = Path(path)
    log_path = path.with_name(path.stem + ".log.npy")
    meta_path = path.with_name(path.stem + ".meta.npz")

    def prog(msg):
        if progress:
            progress(msg)

    if log_path.is_file() and meta_path.is_file():
        prog("mmap cache…")
        meta = np.load(meta_path)
        log = np.load(log_path, mmap_mode="r")
        return {
            "log": log,
            "along_m": np.asarray(meta["along_m"], dtype=np.float64),
            "range_res": float(meta["range_res_m"]),
            "path": path,
            "cached": True,
        }

    prog("decompressing npz (one-time cache)…")
    with np.load(path) as z:
        log16 = np.asarray(z["log"], dtype=np.float16)
        along = np.asarray(z["along_m"], dtype=np.float64)
        range_res = float(z["range_res_m"]) if "range_res_m" in z.files else 0.0165
    prog(f"writing cache {log_path.name}…")
    np.save(log_path, log16)
    np.savez(meta_path, along_m=along, range_res_m=np.float64(range_res))
    del log16
    log = np.load(log_path, mmap_mode="r")
    return {
        "log": log,
        "along_m": along,
        "range_res": range_res,
        "path": path,
        "cached": False,
    }


def compose_like_live_gui(log_img: np.ndarray, knobs: dict) -> np.ndarray:
    """Same order/ops as sonar_control_gui.ss_render_bgr."""
    work = np.asarray(log_img, dtype=np.float32)
    nb = int(knobs["nadir_bins"])
    if nb > 0 and 2 * nb < work.shape[1]:
        work = work.copy()
        c = work.shape[1] // 2
        work[:, c - nb : c + nb] = np.nan
    fs = float(knobs["flatten"])
    if fs > 0.0:
        work = ssi.flatten_across_track(work, fs)
    gray = ssi.log_to_gray(
        work,
        float(knobs["log_min"]),
        float(knobs["log_max"]),
        float(knobs["gamma"]),
    )
    cc = float(knobs["clahe"])
    if cc > 0.0:
        gray = ssi.apply_clahe(gray, cc, _CLAHE_GRID)
        gray = ssi.blank_nadir(gray, nb)
    ds = int(knobs["despeckle"])
    if ds >= 3:
        if ds % 2 == 0:
            ds += 1
        gray = ssi.despeckle_gray(gray, ds)
    return ssi.apply_colormap(gray, knobs["colormap"])


class ExportViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Sidescan export viewer")
        self.geometry("1400x900")

        self.log_raw = None          # float16 mmap
        self.log_f32 = None          # optional contiguous f32 for fast toggles
        self.along_m = None
        self.range_res = 0.0165
        self._path: Path | None = None

        self._bgr = None
        self._scale = 1.0
        self._ox = 0.0
        self._oy = 0.0
        self._photo = None
        self._drag = None
        self._compose_after = None
        self._blit_after = None
        self._load_gen = 0
        self._compose_gen = 0
        self._pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="ssviz")
        self._busy = tk.StringVar(value="")

        root = ttk.Frame(self, padding=6)
        root.pack(fill="both", expand=True)
        left = ttk.Frame(root, width=300)
        left.pack(side="left", fill="y", padx=(0, 6))
        right = ttk.Frame(root)
        right.pack(side="right", fill="both", expand=True)

        ttk.Button(left, text="Open .npz…", command=self.open_npz).pack(fill="x", pady=2)
        ttk.Button(left, text="Reload", command=self.reload).pack(fill="x", pady=2)
        ttk.Button(left, text="Save view…", command=self.save_png).pack(fill="x", pady=2)
        nav = ttk.Frame(left)
        nav.pack(fill="x", pady=4)
        ttk.Button(nav, text="Fit", width=6, command=self.fit_width).pack(side="left", padx=1)
        ttk.Button(nav, text="100%", width=6, command=self.zoom_100).pack(side="left", padx=1)
        ttk.Button(nav, text="−", width=3, command=lambda: self.zoom_by(1 / 1.25)).pack(
            side="left", padx=1
        )
        ttk.Button(nav, text="+", width=3, command=lambda: self.zoom_by(1.25)).pack(
            side="left", padx=1
        )

        self.info = tk.StringVar(value="No file\nWheel=zoom  Drag=pan  Double-click=fit")
        ttk.Label(left, textvariable=self.info, wraplength=280).pack(fill="x", pady=6)
        self.zoom_lbl = tk.StringVar(value="zoom —")
        ttk.Label(left, textvariable=self.zoom_lbl).pack(anchor="w")
        ttk.Label(left, textvariable=self._busy, foreground="#c90").pack(anchor="w", pady=4)

        # Same knobs / ranges / Scale widget as live GUI sidescan tab.
        d = _load_viz_defaults()
        self.vars = {
            "log_min": tk.DoubleVar(value=float(d["log_min"])),
            "log_max": tk.DoubleVar(value=float(d["log_max"])),
            "gamma": tk.DoubleVar(value=float(d["gamma"])),
            "nadir_bins": tk.IntVar(value=int(d["nadir_bins"])),
            "flatten": tk.DoubleVar(value=float(d["flatten"])),
            "clahe": tk.DoubleVar(value=float(d["clahe"])),
            "despeckle": tk.IntVar(value=int(d["despeckle"])),
            "stretch": tk.DoubleVar(value=1.0),
            # Off by default = 1 row per ping (matches live waterfall density).
            "speed_comp": tk.BooleanVar(value=False),
            "colormap": tk.StringVar(value=str(d["colormap"])),
        }

        def add_slider(label, var, lo, hi, res):
            """Match live GUI: tk.Scale + resolution (ttk.Scale breaks IntVars)."""
            row = ttk.Frame(left)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, width=15).pack(side="left")
            tk.Scale(
                row,
                variable=var,
                from_=lo,
                to=hi,
                resolution=res,
                orient="horizontal",
                length=210,
                command=lambda _v: self.schedule_compose(),
            ).pack(side="left", fill="x", expand=True)

        ttk.Checkbutton(
            left, text="Speed compensate", variable=self.vars["speed_comp"],
            command=self.schedule_compose,
        ).pack(anchor="w", pady=4)
        add_slider("Track stretch", self.vars["stretch"], 0.25, 4.0, 0.05)
        add_slider("Log min", self.vars["log_min"], 6.0, 18.0, 0.1)
        add_slider("Log max", self.vars["log_max"], 6.0, 20.0, 0.1)
        add_slider("Gamma", self.vars["gamma"], 0.2, 3.0, 0.05)
        add_slider("Nadir bins", self.vars["nadir_bins"], 0, 512, 1)
        add_slider("Flatten", self.vars["flatten"], 0.0, 1.0, 0.05)
        add_slider("CLAHE clip", self.vars["clahe"], 0.0, 8.0, 0.5)
        add_slider("Despeckle", self.vars["despeckle"], 0, 9, 1)

        cmap_row = ttk.Frame(left)
        cmap_row.pack(fill="x", pady=4)
        ttk.Label(cmap_row, text="Colormap", width=15).pack(side="left")
        cb = ttk.Combobox(
            cmap_row, textvariable=self.vars["colormap"],
            values=["copper", "bronze", "gray"], state="readonly", width=12,
        )
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda *_: self.schedule_compose())

        self.canvas = tk.Canvas(right, bg="#121212", highlightthickness=0, cursor="fleur")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.schedule_blit())
        self.canvas.bind("<MouseWheel>", self._on_wheel)
        self.canvas.bind("<Button-4>", self._on_wheel_linux)
        self.canvas.bind("<Button-5>", self._on_wheel_linux)
        self.canvas.bind("<ButtonPress-1>", self._pan_start)
        self.canvas.bind("<B1-Motion>", self._pan_move)
        self.canvas.bind("<ButtonRelease-1>", self._pan_end)
        self.canvas.bind("<ButtonPress-2>", self._pan_start)
        self.canvas.bind("<B2-Motion>", self._pan_move)
        self.canvas.bind("<ButtonRelease-2>", self._pan_end)
        self.canvas.bind("<Double-Button-1>", lambda e: self.fit_width())
        self.bind("<plus>", lambda e: self.zoom_by(1.25))
        self.bind("<equal>", lambda e: self.zoom_by(1.25))
        self.bind("<minus>", lambda e: self.zoom_by(1 / 1.25))
        self.bind("<KP_Add>", lambda e: self.zoom_by(1.25))
        self.bind("<KP_Subtract>", lambda e: self.zoom_by(1 / 1.25))
        self.bind("<Key-0>", lambda e: self.fit_width())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._default_dir = DEFAULT_DIR if DEFAULT_DIR.is_dir() else Path.home()

    def _on_close(self):
        self._load_gen += 1
        self._compose_gen += 1
        self._pool.shutdown(wait=False, cancel_futures=True)
        self.destroy()

    def _ui(self, fn):
        self.after(0, fn)

    def _set_busy(self, msg: str):
        self._ui(lambda: self._busy.set(msg))

    def _knobs(self) -> dict:
        """Snapshot Tk vars as plain Python types for the compose worker."""
        return {
            "log_min": float(self.vars["log_min"].get()),
            "log_max": float(self.vars["log_max"].get()),
            "gamma": float(self.vars["gamma"].get()),
            "nadir_bins": int(self.vars["nadir_bins"].get()),
            "flatten": float(self.vars["flatten"].get()),
            "clahe": float(self.vars["clahe"].get()),
            "despeckle": int(self.vars["despeckle"].get()),
            "stretch": float(self.vars["stretch"].get()),
            "speed_comp": bool(self.vars["speed_comp"].get()),
            "colormap": str(self.vars["colormap"].get()),
        }

    # ----- file --------------------------------------------------------------

    def open_npz(self):
        path = filedialog.askopenfilename(
            initialdir=str(self._default_dir),
            filetypes=[("Sidescan export", "*_sidescan.npz"), ("NPZ", "*.npz")],
        )
        if path:
            self.load_async(Path(path))

    def reload(self):
        if self._path:
            self.load_async(self._path)

    def load_async(self, path: Path):
        self._load_gen += 1
        self._compose_gen += 1  # cancel in-flight compose for old file
        gen = self._load_gen
        self._path = path
        self._bgr = None
        self.log_f32 = None
        self.canvas.delete("all")
        self.info.set(f"Loading\n{path.name}")
        self._set_busy("loading…")

        def work():
            try:
                def progress(m):
                    if gen == self._load_gen:
                        self._set_busy(m)

                data = load_export(path, progress=progress)
                if gen != self._load_gen:
                    return
                prog = "materializing float32…"
                self._set_busy(prog)
                # Contiguous f32 once — toggles stay fast & correct.
                log_f32 = np.asarray(data["log"], dtype=np.float32)
                if gen != self._load_gen:
                    return

                def apply():
                    self.log_raw = data["log"]
                    self.log_f32 = log_f32
                    self.along_m = data["along_m"]
                    self.range_res = data["range_res"]
                    h, w = log_f32.shape
                    track = (
                        float(self.along_m[-1] - self.along_m[0])
                        if self.along_m.size
                        else 0.0
                    )
                    tag = "cached" if data["cached"] else "cached now"
                    self.info.set(
                        f"{path.name}\n{h}×{w} pings×bins ({tag})\n"
                        f"track≈{track:.1f} m  dx={self.range_res * 100:.1f} cm\n"
                        f"Wheel=zoom  Drag=pan  Double-click=fit"
                    )
                    self._busy.set("")
                    self.schedule_compose(fit=True)

                self._ui(apply)
            except Exception:
                err = traceback.format_exc()
                self._ui(lambda: (
                    self._busy.set("LOAD FAILED"),
                    self.info.set(err[-500:]),
                ))

        self._pool.submit(work)

    def save_png(self):
        if self._bgr is None:
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".jpg",
            filetypes=[("JPEG", "*.jpg"), ("PNG", "*.png")],
            initialfile=(self._path.stem + "_view.jpg") if self._path else "sidescan.jpg",
        )
        if not path:
            return
        bgr = self._bgr
        h, w = bgr.shape[:2]
        if max(h, w) > 65000:
            s = 65000 / float(max(h, w))
            bgr = cv2.resize(
                bgr, (max(int(w * s), 1), max(int(h * s), 1)),
                interpolation=cv2.INTER_AREA,
            )
        cv2.imwrite(path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])

    # ----- compose -----------------------------------------------------------

    def schedule_compose(self, fit: bool = False):
        if self._compose_after:
            self.after_cancel(self._compose_after)
        self._compose_after = self.after(100, lambda: self._compose_async(fit=fit))

    def _compose_async(self, fit: bool = False):
        self._compose_after = None
        if self.log_f32 is None:
            return
        self._compose_gen += 1
        gen = self._compose_gen
        knobs = self._knobs()
        log = self.log_f32
        along = None if self.along_m is None else self.along_m.copy()
        range_res = self.range_res
        self._set_busy("composing…")

        def work():
            try:
                img = log
                if knobs["speed_comp"] and along is not None and along.size == img.shape[0]:
                    img = speed_resample(
                        img, along, range_res,
                        stretch=float(knobs["stretch"]),
                        max_rows=_MAX_COMPOSE_H,
                    )
                # No silent resize when speed-comp is off — keep every ping.
                bgr = compose_like_live_gui(img, knobs)
                if gen != self._compose_gen:
                    return

                def apply():
                    self._bgr = bgr
                    self._busy.set("")
                    bh, bw = bgr.shape[:2]
                    src_h = log.shape[0]
                    mode = "speed-comp" if knobs["speed_comp"] else "1:1 pings"
                    # Keep file line; append compose result so truncation is visible.
                    base = self.info.get().split("\n")
                    head = "\n".join(base[:3]) if len(base) >= 3 else self.info.get()
                    self.info.set(
                        f"{head}\nview {bh}×{bw} ({mode}; src {src_h} pings)\n"
                        f"Wheel=zoom  Drag=pan  Double-click=fit"
                    )
                    if fit or self._scale <= 0:
                        self.fit_width()
                    else:
                        self._clamp_view()
                        self.schedule_blit()

                self._ui(apply)
            except Exception:
                err = traceback.format_exc()
                self._ui(lambda: self._busy.set(f"compose failed\n{err[-240:]}"))

        self._pool.submit(work)

    # ----- view --------------------------------------------------------------

    def schedule_blit(self):
        if self._blit_after:
            self.after_cancel(self._blit_after)
        self._blit_after = self.after(8, self._blit)

    def fit_width(self):
        if self._bgr is None:
            return
        self.update_idletasks()
        cw = max(self.canvas.winfo_width(), 2)
        ch = max(self.canvas.winfo_height(), 2)
        ih, iw = self._bgr.shape[:2]
        self._scale = cw / float(iw)
        self._ox = 0.0
        self._oy = 0.0
        if ih * self._scale < ch:
            self._oy = -((ch / self._scale) - ih) * 0.5
        self._clamp_view()
        self.schedule_blit()

    def zoom_100(self):
        if self._bgr is None:
            return
        self.update_idletasks()
        cw = max(self.canvas.winfo_width(), 2)
        ch = max(self.canvas.winfo_height(), 2)
        self._zoom_at(cw * 0.5, ch * 0.5, 1.0 / max(self._scale, 1e-9))
        self.schedule_blit()

    def zoom_by(self, factor: float):
        if self._bgr is None:
            return
        self.update_idletasks()
        cw = max(self.canvas.winfo_width(), 2)
        ch = max(self.canvas.winfo_height(), 2)
        self._zoom_at(cw * 0.5, ch * 0.5, factor)
        self.schedule_blit()

    def _zoom_at(self, canvas_x: float, canvas_y: float, factor: float):
        new_scale = float(np.clip(self._scale * factor, _ZOOM_MIN, _ZOOM_MAX))
        if abs(new_scale - self._scale) < 1e-12:
            return
        img_x = self._ox + canvas_x / self._scale
        img_y = self._oy + canvas_y / self._scale
        self._scale = new_scale
        self._ox = img_x - canvas_x / self._scale
        self._oy = img_y - canvas_y / self._scale
        self._clamp_view()

    def _clamp_view(self):
        if self._bgr is None:
            return
        ih, iw = self._bgr.shape[:2]
        self.update_idletasks()
        cw = max(self.canvas.winfo_width(), 2)
        ch = max(self.canvas.winfo_height(), 2)
        view_w = cw / self._scale
        view_h = ch / self._scale
        if view_w >= iw:
            self._ox = (iw - view_w) * 0.5
        else:
            self._ox = float(np.clip(self._ox, 0.0, iw - view_w))
        if view_h >= ih:
            self._oy = (ih - view_h) * 0.5
        else:
            self._oy = float(np.clip(self._oy, 0.0, ih - view_h))

    def _on_wheel(self, event):
        if self._bgr is None:
            return
        if event.delta == 0:
            return
        self._zoom_at(event.x, event.y, 1.25 if event.delta > 0 else 1 / 1.25)
        self.schedule_blit()
        return "break"

    def _on_wheel_linux(self, event):
        if self._bgr is None:
            return
        self._zoom_at(event.x, event.y, 1.25 if event.num == 4 else 1 / 1.25)
        self.schedule_blit()
        return "break"

    def _pan_start(self, event):
        self._drag = (event.x, event.y, self._ox, self._oy)
        self.canvas.configure(cursor="hand2")

    def _pan_move(self, event):
        if self._drag is None or self._bgr is None:
            return
        x0, y0, ox0, oy0 = self._drag
        self._ox = ox0 - (event.x - x0) / self._scale
        self._oy = oy0 - (event.y - y0) / self._scale
        self._clamp_view()
        self.schedule_blit()

    def _pan_end(self, _event):
        self._drag = None
        self.canvas.configure(cursor="fleur")

    def _blit(self):
        self._blit_after = None
        if self._bgr is None:
            return
        self.update_idletasks()
        cw = max(self.canvas.winfo_width(), 2)
        ch = max(self.canvas.winfo_height(), 2)
        ih, iw = self._bgr.shape[:2]
        x0 = max(int(np.floor(self._ox)), 0)
        y0 = max(int(np.floor(self._oy)), 0)
        x1 = min(int(np.ceil(self._ox + cw / self._scale)) + 1, iw)
        y1 = min(int(np.ceil(self._oy + ch / self._scale)) + 1, ih)
        if x1 <= x0 or y1 <= y0:
            self.canvas.delete("all")
            return
        crop = self._bgr[y0:y1, x0:x1]
        dw = max(int(round((x1 - x0) * self._scale)), 1)
        dh = max(int(round((y1 - y0) * self._scale)), 1)
        if dw * dh > 12_000_000:
            s = (12_000_000 / (dw * dh)) ** 0.5
            dw = max(int(dw * s), 1)
            dh = max(int(dh * s), 1)
        interp = cv2.INTER_AREA if self._scale < 1.0 else cv2.INTER_LINEAR
        disp = cv2.resize(crop, (dw, dh), interpolation=interp)
        rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        self._photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.canvas.delete("all")
        self.canvas.create_image(
            (x0 - self._ox) * self._scale,
            (y0 - self._oy) * self._scale,
            anchor="nw",
            image=self._photo,
        )
        self.zoom_lbl.set(f"zoom {self._scale * 100:.0f}%   ({iw}×{ih} px)")


def main():
    ExportViewer().mainloop()


if __name__ == "__main__":
    main()
