# Solo sulla copia di prova (MODIFICA=tools/prove-nav/zona-credenza.py tools/prove-nav/costruisci.sh): aggiunge alla zona
# della base una zona proibita sulla credenza rialzata del soggiorno, al posto di quella che metterà l'utente dall'app.
#
# Coordinate della mappa, con il robot partito dalla base del soggiorno (base x=0.6 y=0.22 angolo=90, sim/stanze/soggiorno.txt):
#     x mappa = y stanza - 0.3945        y mappa = -(x stanza - 0.6)
# La credenza va da 4.8 a 5.9 in x e da 2.4 a 3.0 in y nella stanza, quindi da 2.0055 a 2.6055 in x e da -5.3 a -4.2 in y sulla
# mappa; qui con 3 cm di margine per lato.
import sys

p = sys.argv[1] + '/mapping.c'
s = open(p).read()

a = '''    zone_list.num_zones = 1;

    write_forbidden_zone_list(&zone_list);

    mark_forbidden_cells(zone_list.forbidden_zones[0]);
'''
b = '''    zone_list.forbidden_zones[1].x = 1.9755;
    zone_list.forbidden_zones[1].y = -5.33;
    zone_list.forbidden_zones[1].w = 0.66;
    zone_list.forbidden_zones[1].h = 1.16;
    zone_list.num_zones = 2;

    write_forbidden_zone_list(&zone_list);

    mark_forbidden_cells(zone_list.forbidden_zones[0]);
    mark_forbidden_cells(zone_list.forbidden_zones[1]);
'''
assert s.count(a) == 1, 'publish_base_zone è cambiata'
open(p, 'w').write(s.replace(a, b))
