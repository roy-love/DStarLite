namespace DStarLite
{
    internal class StateInfo
    {
        public double[] Keys { get; set; } = { double.PositiveInfinity, double.PositiveInfinity };

        public double G { get; set; } = double.PositiveInfinity;

        public double Rhs { get; set; } = double.PositiveInfinity;

        public double Cost { get; set; } = 1;

        public double CostNew { get; set; }
    }
}
