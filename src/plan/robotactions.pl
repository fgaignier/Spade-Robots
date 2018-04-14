% one object: the cup
object(cup).

% two robots: raphael and samira
robot(samira).
robot(raphael).

% positions: sofa, principale, salle2, chair
position(sofa).
position(principale).
position(salle2).
position(chair).

% support: table, box
support(table).
support(box).
support(chair).

% distance
distance(50).
distance(100).
distance(-70).

% take
can(take(R, X, At, On), [handempty(R), clear(X), at(R,At), at(X,At), on(X, On)], robot) :- 
				object(X), robot(R), position(At), support(On).
adds(take(R, X, At, On), [holding(R,X), free(On)], _ , robot) :- 
			object(X), robot(R), support(On).
deletes(take(R, X, At, On), [handempty(R), clear(X)], robot) :- 
			object(X), robot(R), support(On).

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
			support(X), robot(R), position(At), distance(D).
adds(push(R, X, D), [atD(R, At, D), atD(X, At, D)], _ , robot) :- 
			robot(R), support(X), position(At), distance(D).
deletes(push(R, X, D), [at(R, At), at(X, At)], robot) :- 
			support(X), robot(R), position(At), distance(D).

% move

can(move(R, D), [at(R,At)], robot) :- robot(R), position(At), distance(D).
adds(move(R, D), [atD(R, At, D)], _ , robot) :- robot(R), position(At), distance(D).
deletes(move(R, D), [at(R, At)], robot) :- robot(R), position(At), distance(D).


/*definition de scenaris pour les robots: */
/* premier: getCoffee (prendre le gobelet dans une piece et l'amener dans une autre et le poser sur une boite */

getCoffee(P):- plan([handempty(samira), clear(cup), on(cup, chair), at(samira, principale), at(cup, chair), at(table, sofa), free(table)], [handempty(samira), on(cup, table)], robot, P).
/* second : prendre un objet */
takeObject(P):- plan([handempty(samira), clear(cup), at(samira, principale), at(cup, principale), on(cup,box)], [holding(samira, cup)], robot, P).
/* troisieme : clean room (consiste a bouger une boite en ayant retire l'objet qui est dessus. Doit faire intervenir 2 robots */
clean(P):- plan([handempty(samira), handempty(raphael), clear(cup), on(cup,box), at(cup, principale), at(samira, principale), at(box, principale), at(raphael, salle2)], [atD(box, principale, 100), holding(samira, cup)], robot, P).





