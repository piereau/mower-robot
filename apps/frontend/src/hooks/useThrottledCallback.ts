import { useCallback, useRef } from 'react';

/**
 * Configuration for joystick command rate limiting.
 * 10Hz provides smooth control while saving bandwidth.
 */
export const JOYSTICK_SEND_RATE_HZ = 10;
export const THROTTLE_MS = 1000 / JOYSTICK_SEND_RATE_HZ;

/**
 * Returns a throttled version of a callback that only fires once per interval.
 * Useful for rate-limiting high-frequency events like joystick movement.
 *
 * @param callback - The function to throttle
 * @param delay - Minimum time between calls in milliseconds
 * @returns A throttled callback function
 *
 * @example
 * const throttledMove = useThrottledCallback(
 *   (x, y) => sendCommand({ x, y }),
 *   THROTTLE_MS
 * );
 */
export function useThrottledCallback<T extends (...args: any[]) => void>(
    callback: T,
    delay: number
): T {
    const lastCall = useRef<number>(0);
    const lastArgs = useRef<Parameters<T> | null>(null);
    const timeoutRef = useRef<number | undefined>(undefined);

    return useCallback(
        ((...args: Parameters<T>) => {
            const now = Date.now();
            lastArgs.current = args;

            if (now - lastCall.current >= delay) {
                // Enough time has passed, call immediately
                lastCall.current = now;
                callback(...args);
            } else if (!timeoutRef.current) {
                // Schedule a trailing call for the remaining time
                const remaining = delay - (now - lastCall.current);
                timeoutRef.current = window.setTimeout(() => {
                    lastCall.current = Date.now();
                    timeoutRef.current = undefined;
                    if (lastArgs.current) {
                        callback(...lastArgs.current);
                    }
                }, remaining);
            }
        }) as T,
        [callback, delay]
    );
}
