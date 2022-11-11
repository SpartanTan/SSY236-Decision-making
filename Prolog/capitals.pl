capital(berlin,germany).
capital(athens,greece).
capital(madrid,spain).
start:-write('Which country or capital do you want to know? '),
nl, write('Write the capital or the country in lower letters and end it with a dot. '),
nl,read(A), process(A).
process(A):-capital(B,A),result(B,A).
process(A):-capital(A,B),result(A,B).
result(X,Y):-write(X), write(' is the capital of '),write(Y),write('.'), nl.
