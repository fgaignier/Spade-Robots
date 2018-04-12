object(a).
object(b).
object(c).
robot(samira).
robot(raphael).
position(sofa).
position(principale).
position(chair).
support(table).

% take
can(take(R, X, At), [handempty(R), clear(X), at(R,At), at(X,At)], robot) :- 
				object(X), robot(R), position(At).
adds(take(R, X, At), [holding(R,X)], _ , robot) :- 
			object(X), robot(R).
deletes(take(R, X, At), [handempty(R), clear(X)], robot) :- 
			object(X), robot(R).

% goTo
can(goTo(R, From, To), [at(R, From)], robot) :-
				position(From), position(To), robot(R), From \= To.
adds(goTo(R, From, To), [at(R, To)], _ , robot) :- 
			position(To), robot(R).
deletes(goTo(R, From, To), [at(R,From)], robot) :- 
			position(From), robot(R).

% put
can(put(R, X, On), [holding(R,X), at(R, At), at(On, At), free(On)], robot) :-
			object(X), robot(R), position(At), support(On).
adds(put(R, X, On), [on(X, On), handempty(R), clear(X)], _ , robot) :- 
			support(On), robot(R), object(X).
deletes(put(R, X, On), [holding(R,X), free(On)], robot) :- 
			object(X), robot(R), support(On).

% push
can(push(R, X, D), [handempty(R), at(R, At), at(X, At), free(X)], robot) :-
			object(X), robot(R), position(At).
adds(push(R, X, D), [atD(R, At, D), atD(X, At, D)], _ , robot) :- 
			robot(R), object(X), position(At).
deletes(push(R, X, D), [at(R, At), at(X, At)], robot) :- 
			object(X), robot(R), position(At).

% move
can(move(R, D), [handempty(R), at(R, At), at(X, At), free(X)], robot) :-
			object(X), robot(R), position(At).
adds(move(R, D), [atD(R, At, D), atD(X, At, D)], _ , robot) :- 
			robot(R), object(X), position(At).
deletes(move(R, D), [at(R, At), at(X, At)], robot) :- 
			object(X), robot(R), position(At).


/*definition de scenaris pour les robots: */
/* premier: getCoffee (prendre le gobelet dans une piece et l'amener dans une autre et le poser sur une boite */

getCoffee(P):- plan([handempty(samira), clear(a), at(samira, principale), at(a, chair), at(table, sofa), free(table)], [handempty(samira), on(a, table)], robot, P).
takeObject(P):- plan([handempty(samira), clear(a), at(samira, principale), at(a, principale)], [holding(samira, a)], robot, P).

