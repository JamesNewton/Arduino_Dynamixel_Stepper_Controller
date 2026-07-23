# TODOs

After `?` fails, keep skipping indented lines?

After `?` succeeds, and `!` is being skipped, keep skipping indented lines?

J	(Jump) move NUM lines TODO?

When the pin is an input, H and L set or clear TRUE based on the pins value.

SPI `D('s', MISOpin, MOSIpin, SCKpin)` 

UART / Serial  `D('U', TXpin, RXpin, baud)`


# TO DONEs

## Incremental Blocking String Matcher

Concept: When evaluating incoming text (e.g., t="hello"?), the VM must not accidentally destroy characters if a match fails, so subsequent lines (e.g., t="hi"?) can evaluate them.

The Architecture (Peek-and-Consume):

1. The Custom Ring Buffer: We cannot rely on standard Serial.read() because reading destroys the character. We will implement a custom char rx_buffer[64] that automatically accumulates incoming serial data.

2. Length-Based Blocking: When evaluating t="hello"? (length 5), the VM checks rx_buffer. If it holds fewer than 5 characters, the VM blocks (halts the evaluateABC loop) until enough characters arrive.

3. Non-Destructive Peeking: Once 5 characters are present, the VM peeks at them.

> - If they mismatch, true_flag = false, and the buffer is left completely untouched. The VM moves to the next line.

> - If they match, true_flag = true, and the VM consumes those 5 characters, permanently removing them from the buffer.

The Shared Prefix Trap: We must remember that if command strings share a prefix (e.g., "start" and "starting"), typing the shorter command first will cause a deadlock if the longer command is evaluated first.

### Examples

```
t="one"?1
t="two"?2
t="three"?3
```
etc...

Lets say that's a function and we call it. It has no input text, so it blocks. if we type "t" it should reject the first match, execute to the second line, match the t and then block again. If we type "h" it should reject the second line and match the "th" of the third line and again block, and so on. Key in that is the retention of the "t" we entered to match on the first and second lines, and the "h", and so on.

And now on to the second example, using the same combined method:
```
t="hello"? 1
t="hi"? 2
```
It has no input, so it blocks, waiting. We press "h" so it matches the first h in hello but doesn't have another character so it blocks. We press "i", and the first match fails, the second succeeds, and we are done.