Questo è un "ripetitore" wireless del normale citofono (Farfisa 5 fili).

Il sistema è costituito da una stazione "Base" connessa in "parallelo"
al ricevitore citofonico posto all'ingresso dell'abitazione e da una
stazione "Remote" che può essere posizionata ai piani superiori
dell'abitazione.

Nella fase di "Riposo" la stazione "Base" è configurata come PTX e la stazione
"Remote" è configurata come PRX.

Per verificare se la Remote vuole comunicare con la Base (per esempio per
comandare l'apertura dei cancelli o per verificare acusticamente la presenza
di persone) la Base interroga periodicamente la Remote ed esegue i comandi
che la Remote richiede.

All'arrivo di una "Chiamata Esterna", la stazione Base invia la richiesta
di collegamento e resta in attesa di una risposta per un tempo finito.

In questa fase la Base è ancora PTX e la Remote è ancora PRX.

All'arrivo di una richiesta di collegamento la Remote attiva la "suoneria"
e attende che l'utente accetti la richiesta per un tempo determinato.

Se nessun utente accetta in tempo la richiesta la Base ritorna nella fase di Riposo.

Se invece la Remote risponde in tempo la Base commuta e diviene PRX mentre la
Remote diviene la PTX.

La Base apre in ricezione due "Pipes":
    una per lo scambio delle informazioni sonore e
    l'altra per lo scambio di comandi di "Controllo".

Fintanto che la Remote non chiude la conversazione la Base accetta ed esegue
comandi e suoni dalla Remote e, a sua volta, invia suoni alla Remote.

Quando la conversazione viene chiusa dalla Remote, Base e Remote ritornano
alla configurazione iniziale.
