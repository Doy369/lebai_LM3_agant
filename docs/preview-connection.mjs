// Pages starts offline. Explicit connection is separate from backend authorization.
export class PreviewConnection {
  constructor(fetcher = (...args) => fetch(...args)) {
    this.fetcher = fetcher;
    this.enabled = false;
    this.controller = new AbortController();
  }
  setEnabled(enabled) {
    this.controller.abort();
    this.controller = new AbortController();
    this.enabled = enabled === true;
  }
  async fetch(url, options = {}) {
    if (!this.enabled) throw Error('在线静态预览未连接后端；如需接口功能，请先启用后端连接');
    const session = this.controller;
    const controller = new AbortController();
    const abort = () => controller.abort();
    const signals = [session.signal, options.signal].filter(Boolean);
    signals.forEach(signal => {
      if (signal.aborted) abort();
      else signal.addEventListener('abort', abort, {once: true});
    });
    try {
      const response = await this.fetcher(url, {...options, signal: controller.signal});
      // Parse inside the session so disabling also cancels an unfinished body.
      const payload = await response.json().catch(() => ({}));
      if (controller.signal.aborted || session !== this.controller) throw Error('后端连接已取消');
      if (!response.ok) throw Error(payload.detail || payload.message || response.statusText);
      return payload;
    } finally {
      signals.forEach(signal => signal.removeEventListener('abort', abort));
    }
  }
}
