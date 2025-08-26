# H-stufige Vorhersage als Sequenz von Mixturen
std_msgs/Header header
float32 dt                                   # Zeitschritt [s] zwischen Vorhersagen
builtin_interfaces/Duration[] horizons       # Länge H; relative Zeitpunkte t_k
Mixture2D[] mixtures                         # Länge H; eine Mixture je Zeitschritt