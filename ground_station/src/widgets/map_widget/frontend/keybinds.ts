export type KeybindMap = Record<string, string>;

export type KeybindCallback = () => void;

const MODIFIER_KEYS = new Set([
    "Control",
    "Shift",
    "Alt",
    "Meta",
    "ControlLeft",
    "ControlRight",
    "ShiftLeft",
    "ShiftRight",
    "AltLeft",
    "AltRight",
    "MetaLeft",
    "MetaRight"
]);

/**
 * Normalize a `KeyboardEvent` into the form used by the Python `KeybindManager`.
 *
 * Returns an empty string for bare modifier presses (so pressing Shift alone
 * does not clear or trigger anything).
 */
function keyboardEventToCombo(event: KeyboardEvent): string {
    if (MODIFIER_KEYS.has(event.key)) {
        return "";
    }

    const parts: string[] = [];
    if (event.ctrlKey) {
        parts.push("Ctrl");
    }
    if (event.shiftKey) {
        parts.push("Shift");
    }
    if (event.altKey) {
        parts.push("Alt");
    }
    if (event.metaKey) {
        parts.push("Meta");
    }

    let keyName = event.key;
    if (keyName.length === 1) {
        keyName = keyName.toUpperCase();
    }

    parts.push(keyName);
    return parts.join("+");
}

function combosMatch(a: string, b: string): boolean {
    return a.toLowerCase() === b.toLowerCase();
}

/**
 * Manages keybind registration and dispatch for the map frontend.
 */
export class KeybindHandler {
    private bindings: KeybindMap = {};
    private callbacks: Map<string, KeybindCallback> = new Map();
    private attached = false;

    /**
     * Register a callback for an action.
     *
     * @param action - The action key (e.g. "focus_boat").
     * @param callback - The function to call when the keybind fires.
     */
    register(action: string, callback: KeybindCallback): void {
        this.callbacks.set(action, callback);
    }

    /**
     * Replace the active set of bindings.
     *
     * @param bindings - A map of action key to key-combination string.
     */
    setBindings(bindings: KeybindMap): void {
        this.bindings = { ...bindings };
        this.ensureAttached();
    }

    /**
     * Attach the `keydown` listener to the document. Called automatically on
     * the first `setBindings` call; safe to call multiple times.
     */
    private ensureAttached(): void {
        if (this.attached) {
            return;
        }

        this.attached = true;
        document.addEventListener("keydown", (event) => this.handleKeyDown(event));
    }

    /**
     * The actual keydown handler. Walks the active bindings and fires the
     * first matching callback. Calls `event.preventDefault()` when a binding
     * matches so the browser doesn't also act on the key.
     */
    private handleKeyDown(event: KeyboardEvent): void {
        const combo = keyboardEventToCombo(event);
        if (!combo) {
            return;
        }

        for (const [action, keyCombo] of Object.entries(this.bindings)) {
            if (combosMatch(combo, keyCombo)) {
                const callback = this.callbacks.get(action);
                if (callback) {
                    event.preventDefault();
                    callback();
                    return;
                }
            }
        }
    }
}
