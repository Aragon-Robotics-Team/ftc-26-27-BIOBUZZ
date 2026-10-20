import { copyFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { defineConfig } from 'vite';
import { viteSingleFile } from 'vite-plugin-singlefile';

// `npm run build` makes one self-contained page and copies it to tools/path-planner.html, which is committed so
// anyone can double-click it without npm.
export default defineConfig({
  base: './',
  plugins: [
    viteSingleFile(),
    {
      name: 'copy-to-tools',
      closeBundle() {
        copyFileSync(resolve(import.meta.dirname, 'dist/index.html'), resolve(import.meta.dirname, '../path-planner.html'));
      },
    },
  ],
  // A classic (not module) worker: Chrome won't start module workers from a blob on a file:// page.
  worker: { format: 'iife' },
  build: { target: 'es2022', outDir: 'dist', emptyOutDir: true },
});
