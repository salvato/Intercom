Nella fase di "Riposo" la stazione "Base" è configurata come PTX e le stazioni
"Remote" sono configurate come PRX, ciascuna con il proprio indirizzo.

Per verificare il funzionamento dell'impianto la Base può interrogare
periodicamente le Remote e segnalare quando qualcuna di queste non risponde.

All'arrivo di una "Chiamata Esterna", la stazione Base invia la segnalazione
a ciascuna delle stazioni Remote e resta in attesa che una di esse risponda
alla chiamata.

In questa fase la Base è ancora PTX e le Remote sono ancora PRX. La Base
dopo aver fatto squillare gli "Allerta" su tutti i Remote continua a
interrogarli ciclicamente per un periodo di tempo "T_Allerta".

Se, nel frattempo, la Base riceve altre Chiamate Esterne, fa ricominciare la
procedura di "Allerta" facendo ripartire il conteggio di T_Allerta.

Se entro il tempo T_Allerta nessuna Remote risponde alla chiamata la Base
ritorna nella fase di Riposo.

Quando, invece, una Remote risponde la Base commuta e diviene la PRX mentre la
Remote che ha risposto diviene la PTX.

La Base apre in ricezione due "Pipes":
    una per lo scambio delle informazioni sonore e
    l'altra per lo scambio di comandi di "Controllo".

Fintanto che la Remote non chiude la conversazione la Base accetta ed esegue
comandi e suoni dalla Remote e, a sua volta, invia suoni alla Remote.

Quando la conversazione viene chiusa dalla Remote, Base e Remote ritornano
alla configurazione iniziale.
