**English** · [日本語](./theory.ja.md)

# Theory talk — moving a body with position-based physics

Cast: **Yu**, a middle schooler, and **me**, the one writing this engine.

---

**Yu**: "Hey, isn't this the thing where the puppet moves its own arm? What's going on inside it?"

**Me**: "Glad you asked. Can I start with a failure story first?"

**Yu**: "Out of nowhere?"

**Me**: "So at first, I built the muscles out of springs. I attached a spring to each joint like 'snap back to this angle!' Then the stiffer I made it, the more it thrashed, and at one point the puppet exploded and flew off into space. The arm became a planet."

**Yu**: "Space, huh."

**Me**: "It's not a joke, this is a classic physics-simulation gotcha. And the idea that fixes it is 'position-based physics (XPBD)'. That's what I want to talk about today."

---

## Ordinary physics — "force → acceleration → velocity → position"

**Me**: "You learned this in science class, right? Apply a force to an object, it accelerates. Once it accelerates, it gains velocity. Once it has velocity, its position moves."

**Yu**: "The F=ma thing. Force = mass times acceleration."

**Me**: "Right. Ordinary simulations compute things in that order.

1. What forces are acting on it? (gravity, muscle force, etc.)
2. So what's the acceleration? (force divided by mass)
3. So what's the velocity? (add a little acceleration)
4. So what's the position? (move by that much velocity)

Repeat this frame by frame. Like a flip-book."

**Yu**: "That sounds proper. Why does it explode?"

**Me**: "Because when a spring is stiff, even a tiny deviation makes the 'force' huge. It tries to fix the error with a strong force → overshoots → strong force the other way → overshoots even more... It's like pushing a swing with the wrong timing, and the swing amplitude just keeps growing. That's called 'divergence.' My puppet went to space because of it."

---

## Position-based — "move first, then pull back if it's wrong"

**Yu**: "So what do you do instead?"

**Me**: "Flip the whole idea around. **Stop thinking in terms of force, and think in terms of position instead.**"

**Yu**: "Huh?"

**Me**: "Picture a necklace made of beads on a string. The string has a fixed length. If you yank one bead, there's a moment where the string would have to stretch."

**Yu**: "It goes taut."

**Me**: "What do you do at that moment? **You gently pull the bead back so the string matches its length.** You're not thinking about force or acceleration at all. It's just 'the length is wrong → fix it.' That's the heart of position-based physics.

- Ordinary physics: compute the force → move gradually.
- Position-based: **move the position first → if it broke a rule, correct the position directly.**"

**Yu**: "Correct it — you mean warp it?"

**Me**: "Yeah, it's a bit of a cheat, but you warp it. And that's what makes it so stable. No matter how far off it gets, all you do is 'snap it back to the correct position,' so the force can never run away. Even a stiff string doesn't explode."

**Yu**: "I see, so there's no way to overshoot."

---

## "Constraints" — handling everything with this one idea

**Me**: "And here's the best part. That rule from before, 'keep to the string's length,' is called a **constraint** — a promise that must be kept."

**Yu**: "Constraint."

**Me**: "How many rules do you think you need to move a human body?"

**Yu**: "Um, like, joints shouldn't fall apart?"

**Me**: "Exactly. That's constraint #1. **Joints stay attached.** The bone above the elbow and the bone below it — the point where they connect must never separate. Same as the bead string from before. If it drifts, pull the position back."

**Yu**: "What else?"

**Me**: "Constraint #2: **muscles**. The rule 'stay at this angle.' A promise to hold the arm at this orientation. This one also works by: if the angle is off, rotate toward the target and pull it back."

**Yu**: "Muscles are a rule too? Not a force?"

**Me**: "That's the satisfying part of position-based physics. Muscles can be written as 'a constraint that keeps an angle.' So they don't thrash around like springs."

**Me**: "Constraint #3: **the ground**. 'Don't sink into the table.' If a hand goes inside the table, pull it back up to the tabletop. Again, just a position fix."

**Yu**: "Wait, they're all the same. If it's off, pull it back."

**Me**: "**That.** That's exactly what I wanted to say. Joints, muscles, the ground, even an arm sinking into its own belly — all of it can be handled with **one single idea** called 'constraint.' Inside the engine, they're practically indistinguishable. That's why the code stays so simple. It's the number one reason I chose position-based physics."

---

## Compliance — the softness of a muscle

**Yu**: "But wait, if a muscle is 'absolutely this angle, no exceptions,' wouldn't it move like a stiff robot? People are more, like, floppy."

**Me**: "Sharp catch. That's where **compliance** comes in. In Japanese it translates to 'softness.'"

**Yu**: "Softness."

**Me**: "You attach a dial to the constraint that controls 'how strictly it's enforced.'

- Small compliance = **enforced rigidly** = a strong muscle. Holds the target angle tight.
- Large compliance = **enforced loosely** = a weak muscle. It tries to hold the angle, but loses to gravity and droops down.

You know how someone who's relaxed just slumps down? You can express that with a single dial. Just crank up the number, and the puppet gradually goes limp."

**Yu**: "So a single number gives you a level of fatigue. That's neat."

**Me**: "Make the puppet heavier, and even with the same muscle strength, the arm sags more. A muscular one holds up easily, a floppy one just sinks. It's all a balance between this softness and the weight."

---

## Substeps — fixing it a little at a time, many times

**Yu**: "I'm still stuck on something. Warping the position to fix it — isn't that sloppy? Do you actually know the one, correct position in a single shot?"

**Me**: "You don't. Especially once a bunch of joints are chained together, fixing one throws another off. Fix the shoulder, the elbow drifts; fix the elbow, the wrist drifts..."

**Yu**: "That's whack-a-mole."

**Me**: "Exactly whack-a-mole. So **you don't aim for perfection in one shot**. You fix it a little, fix it a little, and repeat that many times. Gradually everything settles down. This is called a **substep**."

**Yu**: "You do it many times within a single frame?"

**Me**: "Right. You take one frame on screen (1/60th of a second) and slice it into about 20 tiny pieces, and for each tiny piece you do 'move → fix with constraints.' The finer you slice it, the more stable it gets. It's like how you trip if you rush with big strides, but you don't trip if you take small, quick steps."

**Yu**: "I see. So the exploding puppet was taking big strides."

**Me**: "Exactly that. Fine slicing plus position-based physics — those two together keep it stable even with stiff muscles. No more trips to space."

---

## Summary — and where each piece lives in the code

**Yu**: "Let me organize this.

- Ordinary physics is 'force → acceleration → velocity → position.' It explodes when stiff.
- Position-based is 'move first, then pull the position straight back if it's wrong.' Stable.
- Joints, muscles, and the ground are all handled with one single idea: 'constraint.'
- Compliance decides how soft a muscle is. Loose, and it slumps down.
- Substeps fix things a little at a time, many times, for stability.

...did I get that right?"

**Me**: "Perfect. Completely outdone by a middle schooler."

**Me**: "Lastly, here's a one-liner for what each piece of the actual code (`index.js`) corresponds to:

- `Body` … a single bone. An object with mass, position, and orientation.
- `Attach` … constraint #1, 'joints stay attached.' The bead string.
- `Motor` … constraint #2, 'muscle.' Steers toward a target angle. `compliance` is the softness.
- `GroundContact` / `BoxContact` / `Contact` … constraint #3, 'don't sink in.' The table, a tile, your own belly.
- `World.step()` … the main body that slices one frame into `substeps` (default 20), and repeats 'move → fix with constraints.'
- `makeArm` / `makeUpperBody` … these assembled together into an arm or an upper body.

It's all exactly the metaphors we just talked about, no more, no less."

**Yu**: "Then maybe even I could read it."

**Me**: "You could, easily. Go ahead and make it explode once, just for fun."
