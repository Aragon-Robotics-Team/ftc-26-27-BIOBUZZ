/** Tiny DOM helpers for building forms. */

type Child = Node | string | null | undefined | false;

export function h<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs: Record<string, unknown> = {},
  ...children: (Child | Child[])[]
): HTMLElementTagNameMap[K] {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v === undefined || v === null || v === false) continue;
    if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v as EventListener);
    else if (k === 'class') el.className = String(v);
    else if (k === 'style') el.setAttribute('style', String(v));
    else if (k in el && typeof v !== 'string') (el as unknown as Record<string, unknown>)[k] = v;
    else el.setAttribute(k, v === true ? '' : String(v));
  }
  for (const c of children.flat()) {
    if (c === null || c === undefined || c === false) continue;
    el.append(typeof c === 'string' ? document.createTextNode(c) : c);
  }
  return el;
}

/** A number field that commits on change (blur or Enter) and marks itself invalid instead of committing bad input. */
export function num(
  value: number,
  commit: (v: number) => void,
  opts: { min?: number; max?: number; label?: string; disabled?: boolean; cls?: string } = {},
): HTMLInputElement {
  const input = h('input', {
    type: 'text',
    inputmode: 'decimal',
    value: fmt(value),
    class: opts.cls,
    'aria-label': opts.label,
    title: opts.label,
    disabled: opts.disabled,
  });
  input.addEventListener('change', () => {
    const v = Number(input.value.trim());
    const ok = input.value.trim() !== '' && Number.isFinite(v) && (opts.min === undefined || v >= opts.min) && (opts.max === undefined || v <= opts.max);
    input.classList.toggle('invalid', !ok);
    if (ok) commit(v);
  });
  return input;
}

export function fmt(v: number, digits = 3): string {
  if (!Number.isFinite(v)) return '';
  return String(Math.round(v * 10 ** digits) / 10 ** digits);
}

export function text(value: string, commit: (v: string) => boolean | void, opts: { label?: string; placeholder?: string } = {}): HTMLInputElement {
  const input = h('input', { type: 'text', value, 'aria-label': opts.label, placeholder: opts.placeholder });
  input.addEventListener('change', () => {
    const ok = commit(input.value.trim());
    input.classList.toggle('invalid', ok === false);
  });
  return input;
}

export function select<T extends string>(value: T, options: [T, string][], commit: (v: T) => void, label?: string): HTMLSelectElement {
  const el = h('select', { class: 'field', 'aria-label': label }, ...options.map(([v, l]) => h('option', { value: v, selected: v === value }, l)));
  el.addEventListener('change', () => commit(el.value as T));
  return el;
}

export function labelled(label: string, input: HTMLElement): HTMLLabelElement {
  return h('label', { class: 'lbl' }, h('span', {}, label), input);
}

export function toast(message: string, ms = 3500, action?: { label: string; run: () => void }): void {
  for (const old of document.querySelectorAll('.toast')) old.remove();
  const el = h('div', { class: 'toast', role: 'status' }, message,
    action ? h('button', { class: 'toast-action', type: 'button', onclick: () => (el.remove(), action.run()) }, action.label) : null);
  document.body.append(el);
  setTimeout(() => el.remove(), ms);
}
