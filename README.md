# TopoGen

Creates worldwide underlay topologies and outputs them in the following formats:
* JSON
* nodes and edges in txt files
* KML for Google Earth

![image of example topology](https://raw.githubusercontent.com/thillux/TopoGen/master/img/exampleTopology.jpg)

## Dependencies

TopoGen is built via CMake and uses the following libraries:

* Boost
* CGAL
* COIN-OR::LEMON
* GMP
* JsonCpp
* SQLite 3

## Bootstrapping

```bash
./bootstrap.sh
cmake .
make
```

## Running

1) create Google Earth KML
```bash
bin/topoGen --kml
```

2) create node and edge list
```bash
bin/topoGen --graph
```

3) create graph json
```bash
bin/topoGen --json
```

## Contributors

* Michael Grey
* Markus Theil

## Credits

This tool was created at [Ilmenau University of Technology](https://www.tu-ilmenau.de/en/international/).
