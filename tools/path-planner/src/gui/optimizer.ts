/** Starts and stops optimizer runs in a Web Worker. */
import type { Plan, Progress } from '../core/optimize.ts';
import type { Chain, Settings } from '../core/project.ts';
import OptimizerWorker from './worker.ts?worker&inline';

let worker: Worker | null = null;

export function optimize(settings: Settings, chain: Chain, seed: number, onProgress: (p: Progress) => void): Promise<Plan> {
  stop();
  const w = new OptimizerWorker();
  worker = w;
  return new Promise((resolve, reject) => {
    w.onmessage = (e: MessageEvent) => {
      const msg = e.data as { type: 'progress'; progress: Progress } | { type: 'done'; plan: Plan } | { type: 'error'; message: string };
      if (msg.type === 'progress') onProgress(msg.progress);
      else {
        w.terminate();
        if (worker === w) worker = null;
        if (msg.type === 'done') resolve(msg.plan);
        else reject(new Error(msg.message));
      }
    };
    w.onerror = (e) => {
      w.terminate();
      if (worker === w) worker = null;
      reject(new Error(e.message || 'The optimizer stopped unexpectedly'));
    };
    w.postMessage({ settings, chain, seed });
    (w as Worker & { cancel?: () => void }).cancel = () => reject(new Error('stopped'));
  });
}

export function stop(): void {
  if (!worker) return;
  const w = worker as Worker & { cancel?: () => void };
  worker = null;
  w.terminate();
  w.cancel?.();
}
