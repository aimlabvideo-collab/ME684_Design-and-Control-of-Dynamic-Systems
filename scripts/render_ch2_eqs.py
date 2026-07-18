"""Render the Chapter 2 matrix equations as PNGs.

Matrix environments (bmatrix, array) do not survive the kramdown -> MathJax path
used by this site, so the matrix-valued equations in Chapter 2 are embedded as
images instead. This script regenerates them into figures/ch2/.

Usage:  python scripts/render_ch2_eqs.py
"""
import io
import os
import urllib.parse
import urllib.request

from PIL import Image

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "figures", "ch2")

EQS = {
    # Section 2.6 — cart-pole in state-space form
    "eq_ss_linmatrix": r"\begin{bmatrix} M+m & ml_c \\ ml_c & I+ml_c^{2} \end{bmatrix}"
                       r"\begin{bmatrix} \ddot{p} \\ \ddot{\theta} \end{bmatrix}"
                       r"=\begin{bmatrix} F \\ mgl_c\theta \end{bmatrix}",

    "eq_ss_massinv": r"\begin{bmatrix} M+m & ml_c \\ ml_c & I+ml_c^{2} \end{bmatrix}^{-1}"
                     r"=\frac{1}{D}\begin{bmatrix} I+ml_c^{2} & -ml_c \\ -ml_c & M+m \end{bmatrix}",

    "eq_ss_AB": r"A=\begin{bmatrix} 0&0&1&0 \\ 0&0&0&1 \\ 0&a_1&0&0 \\ 0&a_2&0&0 \end{bmatrix}"
                r",\qquad B=\begin{bmatrix} 0 \\ 0 \\ b_1 \\ b_2 \end{bmatrix}",

    "eq_ss_CD": r"C=\begin{bmatrix} 1&0&0&0 \\ 0&1&0&0 \end{bmatrix}"
                r",\qquad D_{\mathrm{ft}}=\begin{bmatrix} 0 \\ 0 \end{bmatrix}",
}


def render(tex):
    """Return a white-background RGB image of `tex`, rendered by CodeCogs."""
    url = ("https://latex.codecogs.com/png.image?"
           + urllib.parse.quote(r"\dpi{600}\huge " + tex, safe=""))
    src = Image.open(io.BytesIO(urllib.request.urlopen(url, timeout=60).read()))
    src = src.convert("RGBA")
    out = Image.new("RGB", src.size, (255, 255, 255))
    out.paste(src, mask=src.split()[3])
    return out


if __name__ == "__main__":
    for name, tex in EQS.items():
        im = render(tex)
        im.save(os.path.join(OUT, name + ".png"))
        print("%s.png  %dx%d" % (name, im.size[0], im.size[1]))
