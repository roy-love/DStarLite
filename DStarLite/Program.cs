using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class Program
    {
        static void Main(string[] args)
        {
            Pathfinder pf = new Pathfinder();
            pf.setGrid(4, 4);
            pf.setStart(0, 0);
            pf.setGoal(3, 3);
            pf.main();
            
        }
    }
}
