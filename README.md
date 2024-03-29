## This is a burst-and-coast model implementation as described in Calovi et al. PLoS Computational Biology article entitled [Disentangling and modeling interactions in fish with burst-and-coast swimming reveal distinct alignment and attraction behaviors](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1005933).


## Dependencies

- [eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [particle-simu](https://github.com/epfl-mobots/particle-simu)

# Usage

- Clone the [particle-simu](https://github.com/epfl-mobots/particle-simu) repository

   `git clone git@github.com:epfl-mobots/particle-simu.git`
   `cd particle-simu`

- Create a folder `exp` (if you don't already have one) and clone this repository

   `mkdir exp`
   
   `git clone git@github.com:epfl-mobots/burst-and-coast.git exp/burst-and-coast`
   
- Configure the environment for the project:
  
   `./waf configure --exp burst-and-coast`

- Compile your project:
  
   `./waf -j --exp burst-and-coast`
   
- Run the example

   `./build/exp/rummy_nose/rummy_sim`
