/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

/**
 * This interface provides a notification callback mechanism. It is used by a component that needs to wait for an
 * event to occur. There are two ways to wait for something to happen. One is to create a TrcEvent and pass it to
 * the component that will signal the event when the operation is completed. Another way is to make yourself
 * implementing the TrcNotificationReceiver interface and pass yourself along to the component that will make a
 * callback to you when the operation is completed.
 *
 * @param <T> specifies the type of the context object.
 */
public interface TrcNotificationReceiver<T>
{
    /**
     * This method is called to notify the occurrence of an event. In general, notification handler should do its
     * processing and return very quickly because the notification may be running on a time critical thread. For
     * example, if this is a timer notification, notify is called on the {@link TrcTimerMgr} thread. Any delay in
     * the notify call will prevent other timers from expiring on-time.
     *
     * @param context specifies the context of the event, can be null.
     */
    void notify(T context);

}   //interface TrcNotificationReceiver
