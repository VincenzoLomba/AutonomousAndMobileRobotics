# Documento di confronto parametri — Tiago Nav2 + Explore Lite

## Riferimenti del confronto

Questo documento assume come riferimenti:

- **OLD Nav2** = `tiago_nav2.yaml` originario
- **Nav2 finale** = `tiago_nav2_pre_shim.yaml` / configurazione finale senza shim
- **Explore Lite attuale** = `params.yaml` del progetto
- **Explore Lite originale** = `explore/config/params.yaml` della repo ufficiale `robo-friends/m-explore-ros2`

Il documento riporta **solo** i parametri effettivamente cambiati e rilevanti, ignorando volutamente tutto ciò che è rimasto identico.

---

# 1. Nav2: differenze reali tra OLD e configurazione finale

## 1.1 Progress checker: da controllo solo traslazionale a controllo posa

Nel file OLD il controller usava `nav2_controller::SimpleProgressChecker` con:

- `required_movement_radius: 0.5`
- `movement_time_allowance: 10.0`

Nel file finale si passa a `nav2_controller::PoseProgressChecker`, con:

- `required_movement_radius: 0.10`
- `movement_time_allowance: 15.0`
- un parametro angolare aggiuntivo impostato a `0.35`

Il senso della modifica è importante: con `SimpleProgressChecker` il robot viene considerato “in progresso” solo se si sposta abbastanza nello spazio; con `PoseProgressChecker` viene invece considerata progresso anche una rotazione sufficiente. Questo va nella direzione giusta per il problema osservato: non si vuole che il robot venga trattato come “fermato” quando in realtà sta cercando di orientarsi prima di partire.

C’è però un caveat serio: nel file finale compare `required_rotation_angle`, non `required_movement_angle`. Per ROS 2 Humble il nome documentato e implementato del parametro del plugin è `required_movement_angle`. Quindi quel parametro scritto così nel file finale ha un rischio concreto di essere ignorato.

Dal punto di vista comportamentale:

- ridurre `required_movement_radius` da `0.5` a `0.10` rende il checker molto meno severo sulla traslazione minima necessaria per considerare valido il progresso
- aumentare `movement_time_allowance` da `10.0` a `15.0` lo rende un po’ più paziente del file OLD, ma resta comunque abbastanza reattivo

In pratica, l’obiettivo della combinazione finale è: non aspettare troppo a lungo in stallo vero, ma nemmeno punire il robot quando fa piccoli assestamenti o una rotazione iniziale.

Un modo corretto di descrivere l’effetto pratico è questo: invece di restare piantato troppo a lungo nello stesso punto, Nav2 può passare prima a dichiarare mancato progresso e a tentare altro, ma senza considerare automaticamente “nessun progresso” una rotazione sul posto sufficientemente significativa.

---

## 1.2 Limiti cinematici del local planner: comportamento molto più cauto

Nel file OLD il DWB era molto più aggressivo:

- `max_vel_x: 1.00`
- `max_speed_xy: 1.00`
- `min_speed_theta: 0.0`
- `acc_lim_x: 2.5`
- `acc_lim_theta: 3.2`
- `decel_lim_x: -2.5`
- `decel_lim_theta: -3.2`
- `vtheta_samples: 20`
- `sim_time: 1.7`
- `trans_stopped_velocity: 0.25`

Nel finale questi valori diventano:

- `max_vel_x: 0.40`
- `max_speed_xy: 0.40`
- `min_speed_theta: 0.15`
- `acc_lim_x: 1.4`
- `acc_lim_theta: 3.0`
- `decel_lim_x: -1.4`
- `decel_lim_theta: -3.0`
- `vtheta_samples: 40`
- `sim_time: 1.5`
- `trans_stopped_velocity: 0.05`

Inoltre:

- `min_y_velocity_threshold` passa da `0.5` a `0.0`

Il significato complessivo è chiaro: il local planner finale è stato reso molto meno veloce e meno impulsivo, con maggiore risoluzione nella ricerca angolare (`vtheta_samples: 40`) e con una soglia molto più bassa per considerare il robot fermo (`trans_stopped_velocity: 0.05`).

Questo spiega bene perché il comportamento finale sia più prudente e meno “nervoso” rispetto all’OLD.

Tra questi parametri, quello che più si collega alla difficoltà sulle rotazioni iniziali è `min_speed_theta: 0.15`: impone una velocità angolare minima non nulla alle traiettorie candidate che ruotano. Però, come osservato sperimentalmente, questo tipo di modifica può aiutare in alcuni casi e peggiorare in altri, perché rende alcune rotazioni più decise ma può anche allargare di più la manovra.

---

## 1.3 Critic del DWB: più peso agli ostacoli, meno peso all’allineamento puro di path

Nel file OLD i pesi erano:

- `BaseObstacle.scale: 0.02`
- `PathAlign.scale: 32.0`
- `GoalAlign.scale: 24.0`
- `PathDist.scale: 32.0`
- `GoalDist.scale: 24.0`
- `RotateToGoal.scale: 32.0`
- `RotateToGoal.slowing_factor: 5.0`

Nel file finale diventano:

- `BaseObstacle.scale: 0.80`
- `PathAlign.scale: 20.0`
- `GoalAlign.scale: 14.0`
- `PathDist.scale: 24.0`
- `GoalDist.scale: 18.0`
- `RotateToGoal.scale: 48.0`
- `RotateToGoal.slowing_factor: 3.0`

Questa è una modifica molto sostanziale.

Nel file OLD il planner locale premiava fortemente l’allineamento al path e alla goal heading, ma quasi non penalizzava la vicinanza agli ostacoli (`BaseObstacle.scale: 0.02` è davvero bassissimo).

Nel finale, invece:

- l’ostacolo pesa molto di più
- i critic di allineamento/path seguono ancora il percorso ma con aggressività ridotta
- `RotateToGoal.scale` sale a `48.0`, quindi la componente che favorisce una corretta chiusura orientazionale pesa di più nel punteggio finale

Questa famiglia di modifiche è coerente con l’obiettivo pratico: evitare che Tiago “tagli” troppo vicino agli ostacoli e dare più importanza alla qualità della rotazione.

Punto importante: `RotateToGoal` non è limitato al goal finale in senso ingenuo. È un critic del DWB e interviene nella valutazione locale delle traiettorie quando quel criterio è rilevante nella fase di moto. Questo spiega perché modificare i suoi pesi possa cambiare il comportamento anche in casi che, visivamente, non sembrano “arrivo al goal finale”.

---

## 1.4 Robot radius: leggera crescita del modello circolare del robot

Nel file OLD `robot_radius` era `0.275` sia nel global che nel local costmap.

Nel finale è `0.29` in entrambi.

L’effetto è semplice: il robot viene rappresentato come leggermente più grande dal punto di vista del collision checking costmap-based.

È una modifica piccola ma coerente con il tentativo di guadagnare un minimo margine di sicurezza in spazi stretti.

---

## 1.5 Inflation layer: raggio più ampio e decadimento diverso

Nel file OLD:

- `inflation_radius: 0.55`
- `cost_scaling_factor: 3.0`

sia nel global che nel local costmap.

Nel finale:

- `inflation_radius: 0.75`
- `cost_scaling_factor: 4.0`

in entrambi.

Qui la distinzione va fatta bene.

### Effetto di `inflation_radius`
Sì, la “banda” inflazionata è davvero più larga nel finale, perché `inflation_radius` passa da `0.55` a `0.75`.

Questo è il parametro che allarga geometricamente la fascia di costo attorno agli ostacoli.

### Effetto di `cost_scaling_factor`
`cost_scaling_factor` non allarga da solo la banda, ma modifica il decadimento esponenziale dei costi dentro quel raggio.

In pratica: nel finale c’è contemporaneamente:

- una fascia più larga
- una distribuzione dei costi più ripida/differente all’interno di quella fascia

Quindi l’impressione visiva che la zona “gonfiata” fosse più ampia è corretta, ma la causa principale è il cambio di `inflation_radius`, non il solo `cost_scaling_factor`.

---

## 1.6 Planner globale: da Navfn a SmacPlanner2D

Nel file OLD il planner globale era `nav2_navfn_planner/NavfnPlanner` con:

- `tolerance: 0.5`
- `use_astar: false`
- `allow_unknown: true`

Nel finale diventa `nav2_smac_planner/SmacPlanner2D` con:

- `tolerance: 0.5`
- `downsample_costmap: false`
- `downsampling_factor: 1`
- `allow_unknown: true`
- `max_iterations: 1000000`
- `max_on_approach_iterations: 1000`
- `max_planning_time: 2.0`
- `cost_travel_multiplier: 2.0`

Questa è una delle modifiche tecnicamente più forti e più sensate.

Smac 2D è un planner globale cost-aware e usa in modo più strutturato l’informazione di costo della costmap.

Punto importante per sviluppi futuri: **alzare ulteriormente `cost_travel_multiplier` sarebbe un ottimo prossimo passo se l’obiettivo è ottenere traiettorie globali meno fluide ma molto più lontane dagli ostacoli**.

È una leva molto coerente con l’obiettivo di privilegiare la clearance rispetto all’eleganza del path.

---

# 2. Explore Lite: differenze reali tra upstream ufficiale e file attuale

## 2.1 `planner_frequency`: da `0.15` a `0.10`

Nel file ufficiale upstream di `m-explore-ros2`:

- `planner_frequency: 0.15`

Nel file attuale:

- `planner_frequency: 0.10`

Questo rende Explore Lite meno frequente nel ripianificare.

In pratica:

- riduce il churn dei goal
- lascia più tempo al robot per portare avanti il goal corrente prima di cambiare decisione

È una modifica coerente con il tentativo di evitare replanning troppo nervoso.

---

## 2.2 `progress_timeout`: da `30.0` a `60.0`

Nell’upstream ufficiale:

- `progress_timeout: 30.0`

Nel file attuale:

- `progress_timeout: 60.0`

Questa è una modifica molto mirata:

Explore Lite diventa molto più paziente prima di giudicare fallito un tentativo di avanzamento verso una frontier.

È perfettamente coerente con il problema osservato più volte, cioè:

- frontier corrette ma lontane
- oppure goal ragionevoli che richiedono molto tempo prima di produrre nuova mappa utile

---

## 2.3 `min_frontier_size`: da `0.75` a `1.0`

Nell’upstream ufficiale:

- `min_frontier_size: 0.75`

Nel file attuale:

- `min_frontier_size: 1.0`

Questa modifica rende Explore Lite più selettivo:

- frontiere piccole vengono scartate più facilmente

### Vantaggio
Filtra frontier spurie o troppo piccole.

### Svantaggio
Si rischia di ignorare aperture piccole ma reali o lembi di ignoto ancora esplorabili.

Quindi, se in futuro il problema diventasse “restano zone esplorabili non visitate”, **rialzarlo ancora non sarebbe la direzione naturale**. Più facilmente si valuterebbe di non spingerlo troppo in alto o persino di riavvicinarlo al valore upstream.

---

## 2.4 `min_prerotation_angle: 0.1745` è una estensione custom

Nel file attuale compare:

- `min_prerotation_angle: 0.1745`

con commenti espliciti sulla pre-rotazione prima del `NavigateToPose`.

Questo parametro **non è presente** nel `params.yaml` ufficiale della repo `m-explore-ros2`, quindi è una modifica locale/custom del progetto.

Il suo effetto, come da commento nel file, è introdurre una soglia minima sotto la quale la pre-rotazione viene saltata.

È una modifica molto coerente con il problema della partenza verso goal che richiedono un’inversione marcata di heading.

---

# 3. Nota finale di controllo: gli altri parametri sono identici?

Per questo confronto, la risposta corretta è:

## Nav2
Sì, fuori dai blocchi descritti sopra, non risultano altre differenze sostanziali tra l’OLD scelto e il file finale di riferimento.

Le altre differenze residue sono di:

- formattazione YAML
- commenti
- stile

non di contenuto operativo.

## Explore Lite
Sì, rispetto al `params.yaml` upstream ufficiale, i cambi che emergono nel file attuale sono quelli riportati sopra.

Gli altri parametri standard presenti coincidono.

---

# 4. Punto tecnico più importante da correggere

Il punto davvero più importante da portarsi via è questo:

**nel file Nav2 finale il passaggio concettuale a `PoseProgressChecker` è giusto, ma il nome del parametro angolare va corretto in `required_movement_angle` per essere coerente con Nav2 Humble.**

Questo è il dettaglio tecnico più delicato emerso dal confronto.
