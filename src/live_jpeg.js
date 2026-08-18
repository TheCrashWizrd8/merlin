/* Latest-frame JPEG viewer for Firefox.
 * Long-poll /snapshot?since=N so we do not re-download the same JPEG.
 * Canvas blit — Firefox queues <img> / MJPEG by about a second. */
function attachLiveJpeg(el, url) {
  url = url || "/snapshot";
  const canvas = el.tagName === "CANVAS" ? el : (function () {
    const c = document.createElement("canvas");
    c.id = el.id;
    c.className = el.className;
    c.setAttribute("aria-label", el.getAttribute("alt") || "Live stream");
    el.replaceWith(c);
    return c;
  })();
  const ctx = canvas.getContext("2d", { alpha: false });
  if (!ctx) return;
  let since = 0;

  async function blit(blob) {
    const bmp = await createImageBitmap(blob);
    if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
      canvas.width = bmp.width;
      canvas.height = bmp.height;
    }
    ctx.drawImage(bmp, 0, 0);
    bmp.close();
  }

  async function pull() {
    for (;;) {
      try {
        const res = await fetch(
          url + "?since=" + since + "&t=" + Date.now(),
          { cache: "no-store" }
        );
        if (!res.ok) throw new Error("http " + res.status);
        const gen = Number(res.headers.get("X-Frame-Gen"));
        if (gen > since) since = gen;
        await blit(await res.blob());
      } catch (err) {
        await new Promise(function (r) { setTimeout(r, 80); });
      }
    }
  }

  pull();
}
