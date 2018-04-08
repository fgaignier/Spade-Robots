object(a).
object(b).
object(c).
robot(samira).
position(smallbox).
position(salle1).
position(salle2).
support(box).

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



/*definition de scenaris pour les robots: */
/* premier: getCoffee (prendre le gobelet dans une piece et l'amener dans une autre et le poser sur une boite */

getCoffee(P):- plan([handempty(samira), clear(a), at(samira, salle1), at(a, smallbox), at(box, salle2), free(box)], [handempty(samira), on(a,box)], robot, P).


