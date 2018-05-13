% one object: the cup
object(cup).
object(cup2).

% two robots: raphael and samira
robot(samira).
robot(raphael).

% positions:
position(start).
position(sofa).
position(tv).
position(box).

% support:
support(box1).
support(box2).
support(box3).

/* CAN : PRECONDITIONS */
% take
can(take(R, X, At, On), [handempty(R), at(R,At), at(On, At), on(X, On)], robot) :- 
				object(X), robot(R), position(At), support(On).
% put
can(put(R, X, On), [holding(R,X), at(R, At), at(On, At), free(On)], robot) :-
			object(X), robot(R), position(At), support(On).

% goTo
can(goTo(R, From, To), [at(R, From), accessible(To)], robot) :-
			position(From), position(To), robot(R), From \= To.
% push
can(push(raphael, X, 40), [handempty(raphael), at(raphael, At), at(X, At), free(X)], robot) :-
			support(X), position(At).

/* ADDS : POSITIVE EFFECTS */
% take
adds(take(R, X, At, On), [holding(R,X), free(On)], _ , robot) :- 
			object(X), robot(R), position(At), support(On).
% put
adds(put(R, X, On), [on(X, On), handempty(R)], _ , robot) :- 
			support(On), robot(R), object(X).
% goTo
adds(goTo(R, From, To), [at(R, To), accessible(From)], _ , robot) :- 
			position(To), position(From), robot(R), From \= To.
% push
adds(push(raphael, X, 40), [cleared(X)], _ , robot) :- 
			support(X).

/* DELETES : NEGATIVE EFFECTS */
% take
deletes(take(R, X, At, On), [handempty(R)], robot) :- 
			object(X), robot(R), position(At), support(On).

% put
deletes(put(R, X, On), [holding(R,X), free(On)], robot) :- 
			object(X), robot(R), support(On).

% goTo
deletes(goTo(R, From, To), [at(R,From), accessible(To)], robot) :- 
			position(From), position(To), robot(R), From \= To.
% push
deletes(push(raphael, X, 40), [], robot) :- 
			support(X).


/* Starting situation */
/* getCoffee() */
getCoffee(P):- plan([at(box1, box), at(box2, tv), at(samira, start), at(raphael, tv), handempty(samira), on(cup, box1), free(box2), accessible(box)], [at(samira, tv), on(cup, box2)], robot, P), assert(plan_getCoffee(P)).

/* clear box1 */
clearBox1(P):- plan([at(box1, box), at(box2, tv), at(samira, start), at(raphael, tv), handempty(samira), handempty(raphael), on(cup, box1), accessible(box)], [holding(samira, cup), cleared(box1)], robot, P), assert(plan_clearBox1(P)).

/* move cups */
moveCups(P):- plan([at(box1, box), at(box2, tv), at(box3, sofa), at(samira, start), at(raphael, tv), handempty(samira), handempty(raphael), on(cup, box1), on(cup2, box2), accessible(box), accessible(sofa), free(box3)], [on(cup, box3), on(cup2, box1)], robot, P), assert(plan_moveCups(P)).








