/// <reference lib="webworker" />
/** Runs the optimizer off the page's thread. Stopping is done by terminating the worker. */
import { optimizeChain } from '../core/optimize.ts';
import type { Chain, Settings } from '../core/project.ts';

self.onmessage = (e: MessageEvent<{ settings: Settings; chain: Chain; seed: number }>) => {
  try {
    const plan = optimizeChain(e.data.settings, e.data.chain, { seed: e.data.seed, onProgress: (p) => void self.postMessage({ type: 'progress', progress: p }) });
    self.postMessage({ type: 'done', plan });
  } catch (err) {
    self.postMessage({ type: 'error', message: err instanceof Error ? err.message : String(err) });
  }
};
