# GigaProsjekt

## Plan

FRIST 30.5 KL 12:00

Hovedansvar: 
* Få opp kamera fysisk: JA og A
* ROS2, ryddig noder etc: ALLE
* Ekstrafunksjoner: K og Ø
* Teori underveis: ALLE NOTER UNDERVEIS
* Bildeprossesering, finne pos: JA og A
* Kontrollere robot (inverskinematikk, peke etc.): K og Ø
* Error farge/kube: JA og A


24.4 - 27.4: 
* Få opp ROS2 (Ø)
* Fordele oppgaver / lage plan
* Få igang kamera, research fargedeteksjon (opencv), feste til robot (JA og A)
* Prøve å koble på robot, teste fysisk greier (K og Ø)

28.4 - 4.5:
* 

5.5 - 11.5:

12.5 - 18.5:
* BLI FERDIG / STARTE PÅ RAPPORT

19.5 - 25.5:

26.5 - 30.5:

## Møtelogg

# 23.05

Oppmøte: Alle

Hva er gjort?
* Programatisk kontroll av robot
* Detektere ulike objekter med ulike farger

Hva skal gjøres?
* Inverskinematikk
* Montere kamera med feste
* Gjøre om deteksjonskode til ROS

Frist/Neste møte: 26.05

# 26.05

Oppmøte: Alle

Hva er gjort?
* Programatisk kontroll av robot
* Inverse Kinematics
* Detektere ulike objekter med ulike farger

Hva skal gjøres?
* Error detection om det mangler en farge (A)
* Bindeledd kamera og IK (JA)
* Topics mellom kamera og IK (K)
* Rapport disposisjon (Ø)

## Oppgaven
Dere skal kunne flytte roboten til en hjemposisjon, som er en nøytral posisjon (for eksempel 90° knekk på albue). 

Dere skal kunne kjøre en node, en launch-fil eller en kommando (via GUI eller terminal) som flytter roboten dit.

Roboten skal kunne gjøre en serie med oppgaver når den starter:
* Flytte seg til å kunne ta et oversiktsbilde over arbeidsbordet.
* Detektere tre kuber i forskjellige farger fra dette bildet (foreløpig rød, gul og blå, men fargen kan variere).
* Roboten skal så bevege seg nært (typ 10 cm, men det trengs ikke være nøyaktig) til først den røde, så den gule så den blå. Roboten kan bevege seg via et mellompunkt mellom de tre.
* Hvis roboten ikke detekterer en farge, skal den prøve å lete etter den. Det kan være så enkelt som å flytte til en ny posisjon og ta bilde, men kan også være at den tar en serie med bilder fra flere posisjoner.
* Hvis den fortsatt ikke finner kuben skal den stoppe og gi en varsel.

Dere kan som sagt få ekstra poeng ved å utvide disse kriteriene, og også å fjerne noen for å erstatte det med noe annet. Dette må i så fall rettferdiggjøres, og det er lurt om dere spør først.


## Vurderingsgrunnlag

ROS2 (6 poeng):
* Hvor godt er nodene strukturert i forhold til hverandre
* Hvor godt er nodene implementert (riktig bruk av launch, config, osv.)
* Hvor konfigurerbart er det (bruk av parameter, argumenter i launch, o.l.)

Kamera og robot (10 poeng):
* Hvordan er kamera-pipeline satt sammen?
* Hvor robust er kamera til å detektere kubene (forskjellige posisjoner, høyder, lysforhold, osv.)
* Hvordan beveger roboten seg (hvor presist, hvordan peker den, korrigerer den underveis, o.l.)

Skriving og teori (10 poeng):
* Hvor godt skrevet/strukturert er rapporten
* Hvor godt vurderes metoder, implementering o.l. opp mot hverandre
* Hvor godt er teoriforståelse i faget 

Samarbeid (6 poeng):
* Hvor godt samarbeider gruppen, og hvordan var arbeidsfordelingen
* Hvilke verktøy er brukt innad i gruppen (git, overleaf, discord, osv.) 

Ekstra (8 poeng):

Det tildeles ekstra poeng for det som gjøres utover kravene til oppgaven.
Det blir sett etter om dere gir robotsystemet tilleggsfunksjoner som for eksempel: 
* programmerte sikkerhetssoner
* deteksjon av feil tilstand (for eksempel at en farge mangler),
* plukke opp og sortere kuber,
* bruke noe annet enn kuber,
* oppkobling mot PLS eller andre sensorer eller motorer,
* bruk av kraftstyring,
* bygge noe rundt, osv.
* Hvis dere velger å forme løsningen rundt en industriell problemstilling (del av en produksjonsprosess, løse et problem for en bedrift, e.l.), vil det også telle.
Det gjør ingenting om dere må gjøre forenklinger eller gjør en dårligere jobb enn en reell industriell løsning, så lenge det rettferdiggjøres i rapporten.

Dere trenger ikke å gjøre alt (altså ekstra), og det vil vurderes ut ifra kvalitet(!!!). Så en godt implementert ekstrafunksjon er bedre enn ti dårlig implementerte
Med i denne vurderingen vil også anvendelse og relevans bli tatt i betraktning.
