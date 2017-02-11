using System;

namespace DStarLite
{
    internal class Program
    {
        private static void Main()
        {
            var pf = new Pathfinder();
            pf.SetGrid(10, 10);
            //pf.setStart(0, 0);
            //pf.setGoal(3, 3);
            //pf.main();
            pf.SetStart(3, 3);
            pf.SetGoal(0, 0);
            pf.Main();
        }
    }
}
