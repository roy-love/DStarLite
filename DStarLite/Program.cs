using System;

namespace DStarLite
{
    internal class Program
    {
        private static void Main()
        {
            var pf = new Pathfinder();
            pf.SetGrid(4, 4);
            //pf.setStart(0, 0);
            //pf.setGoal(3, 3);
            //pf.main();
            pf.SetStart(3, 3);
            pf.SetGoal(0, 0);
            pf.UpdateCost(1, 1, 1000);
            pf.Main();
            Console.ReadLine();
        }
    }
}
