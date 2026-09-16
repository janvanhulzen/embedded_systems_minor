# Ubuntu realtime-permissies voor ROS 2-experimenten

## Doel

In sommige ROS 2-experimenten willen we de prioriteit van een proces of thread verhogen. Dit is bijvoorbeeld nodig wanneer een executor-thread met `SCHED_FIFO` moet draaien om timinggedrag, latency of jitter te onderzoeken.

Een normale gebruiker mag in Ubuntu standaard niet zomaar realtime-prioriteiten instellen. Daarom moeten we de gebruiker eerst toestemming geven om `rtprio` te gebruiken.

> Let op: realtime-prioriteit kan je systeem traag maken of zelfs tijdelijk onbruikbaar maken als een programma niet meer blokkeert of slaapt. Gebruik dit dus alleen voor gecontroleerde experimenten.

---

## Achtergrond

In C++ kan een programma de scheduler-policy en prioriteit aanpassen met bijvoorbeeld:

```cpp
sched_param sch;
sch.sched_priority = 80;

if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
  throw std::runtime_error{
    std::string("failed to set scheduler: ") +
    std::strerror(errno)};
}
```

Hier betekent:

| Onderdeel | Betekenis |
|---|---|
| `SCHED_FIFO` | Linux realtime scheduler-policy |
| `sched_priority = 80` | realtime-prioriteit van de thread/process-context |
| `sched_setscheduler(...)` | systeemaanroep om scheduler en prioriteit in te stellen |
| `0` als eerste argument | pas dit toe op de huidige thread/process-context |

De prioriteit die je in de code gebruikt, mag niet hoger zijn dan wat de gebruiker via `rtprio` is toegestaan.

---

## Stap 1 — Maak een realtime-groep aan

Open een terminal en voer uit:

```bash
sudo groupadd realtime
```

Als de groep al bestaat, krijg je mogelijk een melding zoals:

```text
groupadd: group 'realtime' already exists
```

Dat is geen probleem.

---

## Stap 2 — Voeg jezelf toe aan de realtime-groep

Voer uit:

```bash
sudo usermod -aG realtime $USER
```

Controleer eventueel je groepen:

```bash
groups
```

Let op: de nieuwe groepsrechten zijn pas actief na opnieuw inloggen. Log dus uit en weer in, of herstart de computer.

```bash
reboot
```

---

## Stap 3 — Maak een limits-bestand aan

Maak een nieuw configuratiebestand:

```bash
sudo nano /etc/security/limits.d/99-realtime.conf
```

Zet daarin:

```text
@realtime - rtprio 98
@realtime - memlock unlimited
@realtime - nice -20
```

Sla het bestand op.

In `nano` doe je dat met:

```text
Ctrl+O   opslaan
Enter    bevestigen
Ctrl+X   afsluiten
```

---

## Betekenis van deze instellingen

| Regel | Betekenis |
|---|---|
| `@realtime - rtprio 98` | Gebruikers in de groep `realtime` mogen realtime-prioriteiten tot 98 gebruiken. |
| `@realtime - memlock unlimited` | Gebruikers mogen geheugen vastzetten, nuttig bij `mlockall()`. |
| `@realtime - nice -20` | Gebruikers mogen processen een hogere normale Linux-prioriteit geven via nice-values. |

Voor het ROS 2-practicum is vooral deze regel belangrijk:

```text
@realtime - rtprio 98
```

---

## Stap 4 — Log opnieuw in

Log volledig uit en weer in, of herstart de computer:

```bash
reboot
```

Daarna zijn de nieuwe permissies actief.

---

## Stap 5 — Controleer de instellingen

Controleer de maximaal toegestane realtime-prioriteit:

```bash
ulimit -r
```

Verwachte output:

```text
98
```

Controleer ook of memory locking is toegestaan:

```bash
ulimit -l
```

Als dit `unlimited` geeft, is memory locking toegestaan.

---

## Stap 6 — Test met `chrt`

Met `chrt` kun je testen of je als gewone gebruiker een programma met realtime-prioriteit mag starten.

```bash
chrt -f 80 sleep 5
```

Als dit zonder foutmelding werkt, mag je `SCHED_FIFO` met prioriteit 80 gebruiken.

Je kunt ook een ROS 2-programma met realtime-prioriteit starten:

```bash
chrt -f 80 ros2 run br2_deep_ros wakeup
```

Of bijvoorbeeld:

```bash
chrt -f 80 ros2 run br2_deep_ros executors
```

Als de C++-code zelf al `sched_setscheduler()` gebruikt, is `chrt` niet per se nodig. Het programma heeft dan vooral de permissie nodig om zichzelf of een thread naar `SCHED_FIFO` te zetten.

---

## Veilige waarden voor het practicum

Gebruik niet meteen prioriteit 99. Laat ruimte over voor kernel- en systeemprocessen.

Een geschikte onderwijsinstelling is bijvoorbeeld:

```text
Toegestane rtprio:     98
Gebruikte experiment-priority: 70 of 80
```

Gebruik in de code bijvoorbeeld:

```cpp
sch.sched_priority = 80;
```

---

## Belangrijke waarschuwing

Een `SCHED_FIFO`-thread met hoge prioriteit kan blijven draaien zolang hij niet blokkeert, slaapt of door een nog hogere realtime-prioriteit wordt onderbroken.

Vermijd daarom in realtime-threads:

- eindeloze busy loops zonder `sleep` of blocking call;
- zware logging naar terminal of bestand;
- langdurige callbacks;
- onnodige mutex-locks;
- dynamische geheugenallocatie in kritische callbacks.

Gebruik realtime-prioriteiten alleen voor korte, gecontroleerde experimenten.

---

## Samenvatting

```text
1. Maak een groep realtime aan.
2. Voeg je gebruiker toe aan deze groep.
3. Geef de groep rtprio-rechten via /etc/security/limits.d/.
4. Log opnieuw in of reboot.
5. Controleer met ulimit -r.
6. Test met chrt of met ROS 2-code die sched_setscheduler() gebruikt.
```

Kernidee:

```text
Linux staat gewone gebruikers standaard niet toe om high-priority
realtime threads te maken.

Met rtprio-permissies kan een ROS 2-programma geselecteerde executor-threads
met SCHED_FIFO-prioriteit laten draaien.
```
