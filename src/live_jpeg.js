/* Latest-frame JPEG viewer.
 * Long-poll /snapshot/<channel>?since=N — canvas blit avoids Firefox MJPEG lag. */
function attachLiveJpeg(el, url, options) {
  options = options || {};
  url = url || '/snapshot/fov';
  const overlayUrl = options.overlayUrl || null;
  const overlaySource = options.overlaySource || 'fov';
  const canvas = el.tagName === 'CANVAS' ? el : (function () {
    const c = document.createElement('canvas');
    c.id = el.id;
    c.className = el.className;
    c.setAttribute('aria-label', el.getAttribute('alt') || 'Live stream');
    el.replaceWith(c);
    return c;
  })();
  const ctx = canvas.getContext('2d', { alpha: false });
  if (!ctx) return { stop: function () {} };

  let since = 0;
  let stopped = false;
  const minIntervalMs = 50;
  const fitMode = options.fit || 'native'; /* native | cover | contain */
  let lastBmp = null;
  let lastOverlay = null;
  let lastFit = null;

  function displayHost() {
    return canvas.closest('.hud-camera') || canvas.parentElement || canvas;
  }

  function displaySize() {
    const host = displayHost();
    return {
      w: Math.max(1, host.clientWidth || window.innerWidth || 1),
      h: Math.max(1, host.clientHeight || window.innerHeight || 1),
    };
  }

  function fitTransform(srcW, srcH, dstW, dstH, mode) {
    if (!srcW || !srcH) {
      return { scale: 1, dx: 0, dy: 0, dw: dstW, dh: dstH };
    }
    const scale = mode === 'contain'
      ? Math.min(dstW / srcW, dstH / srcH)
      : Math.max(dstW / srcW, dstH / srcH);
    const dw = srcW * scale;
    const dh = srcH * scale;
    return {
      scale: scale,
      dx: (dstW - dw) / 2,
      dy: (dstH - dh) / 2,
      dw: dw,
      dh: dh,
    };
  }

  function drawOverlay(payload, fit) {
    if (!payload || !overlayUrl || !fit || !lastBmp) return;
    const src = payload[overlaySource] || payload.fov;
    if (!src) return;
    const srcW = src.width || lastBmp.width;
    const srcH = src.height || lastBmp.height;
    if (!srcW || !srcH) return;
    const sx = fit.scale * (lastBmp.width / srcW);
    const sy = fit.scale * (lastBmp.height / srcH);
    const ox = fit.dx;
    const oy = fit.dy;
    const items = (src.objects || []).concat(src.track ? [src.track] : []);
    ctx.lineWidth = Math.max(2, Math.round(2 * Math.max(sx, 1)));
    items.forEach(function (obj) {
      if (!obj) return;
      const x1 = (obj.x1 || 0) * sx + ox;
      const y1 = (obj.y1 || 0) * sy + oy;
      const x2 = (obj.x2 || 0) * sx + ox;
      const y2 = (obj.y2 || 0) * sy + oy;
      const isTrack = src.track && obj === src.track;
      ctx.strokeStyle = isTrack ? '#22c55e' : '#3b82f6';
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      if (obj.label) {
        ctx.fillStyle = isTrack ? '#22c55e' : '#93c5fd';
        ctx.font = Math.max(12, Math.round(12 * Math.max(sx, 1))) + 'px system-ui,sans-serif';
        ctx.fillText(obj.label, x1 + 2, Math.max(12, y1 - 2));
      }
    });
  }

  function redraw() {
    if (!lastBmp) return;
    if (fitMode === 'native') {
      if (canvas.width !== lastBmp.width || canvas.height !== lastBmp.height) {
        canvas.width = lastBmp.width;
        canvas.height = lastBmp.height;
      }
      ctx.drawImage(lastBmp, 0, 0);
      lastFit = {
        scale: canvas.width / lastBmp.width,
        dx: 0,
        dy: 0,
      };
      drawOverlay(lastOverlay, lastFit);
      return;
    }
    const { w, h } = displaySize();
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, w, h);
    lastFit = fitTransform(lastBmp.width, lastBmp.height, w, h, fitMode);
    ctx.drawImage(lastBmp, lastFit.dx, lastFit.dy, lastFit.dw, lastFit.dh);
    drawOverlay(lastOverlay, lastFit);
  }

  async function blit(blob, overlayPayload) {
    const bmp = await createImageBitmap(blob);
    if (lastBmp) lastBmp.close();
    lastBmp = bmp;
    lastOverlay = overlayPayload;
    redraw();
  }

  if (fitMode !== 'native' && 'ResizeObserver' in window) {
    const ro = new ResizeObserver(function () { redraw(); });
    ro.observe(displayHost());
  }
  if (fitMode !== 'native') {
    window.addEventListener('resize', redraw);
  }

  async function pull() {
    for (; !stopped;) {
      const t0 = performance.now();
      let overlayPayload = null;
      try {
        if (overlayUrl) {
          try {
            const ores = await fetch(overlayUrl, { cache: 'no-store' });
            if (ores.ok) overlayPayload = await ores.json();
          } catch (_) { /* overlay optional */ }
        }
        const res = await fetch(
          url + '?since=' + since + '&t=' + Date.now(),
          { cache: 'no-store' }
        );
        if (!res.ok) throw new Error('http ' + res.status);
        const gen = Number(res.headers.get('X-Frame-Gen'));
        if (gen > since) since = gen;
        await blit(await res.blob(), overlayPayload);
        const leftover = minIntervalMs - (performance.now() - t0);
        if (leftover > 0) {
          await new Promise(function (r) { setTimeout(r, leftover); });
        }
      } catch (err) {
        await new Promise(function (r) { setTimeout(r, 80); });
      }
    }
  }

  pull();
  return {
    stop: function () { stopped = true; },
  };
}

/** Start a stream when `root` scrolls into view; stop when it leaves. */
function attachLazyLiveJpeg(root, selector, url, options) {
  const el = root.querySelector(selector);
  if (!el || !('IntersectionObserver' in window)) {
    return attachLiveJpeg(el || root, url, options);
  }
  let handle = null;
  const obs = new IntersectionObserver(function (entries) {
    const vis = entries.some(function (e) { return e.isIntersecting; });
    if (vis && !handle) {
      handle = attachLiveJpeg(el, url, options);
    } else if (!vis && handle) {
      handle.stop();
      handle = null;
    }
  }, { rootMargin: '120px', threshold: 0.05 });
  obs.observe(root);
  return {
    stop: function () {
      obs.disconnect();
      if (handle) handle.stop();
      handle = null;
    },
  };
}
