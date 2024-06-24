# sdrg
An efficient implementation of the Strong Disorder Renormalisation Group algorithm to treat transverse field Ising models.
Requires C++ Boost Graph Library.

1. '/src' contains all the .cpp and .hpp files.

4. Compilation is handled with the 'Makefile'.

3. '/build' holds temporary build files. Can be cleared using 'make clean'.

5. The 'config.txt' file provides neccesary parameters.

6. A JSON file with the state of the graph can be printed into a 'graph_data.json' file.

7. JSON files in /json can be visualised with  'plotter.ipynb'

8. 'run.sh' isolates the executable and config files into a new unique folder.

9. 'parallel.sh' parallelises 'run.sh' using GNU Parallel.

10. Enjoy!
