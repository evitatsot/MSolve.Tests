using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
//using ISAAR.MSolve.Analyzers;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.NonLinear;
//using ISAAR.MSolve.Analyzers.NonLinear;
//using ISAAR.MSolve.Discretization;
using MGroup.NumericalAnalyzers.Discretization;
using MGroup.FEM;
using MGroup.FEM.Structural;
using MGroup.FEM.Structural.Tests;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Integration;
using MGroup.Solvers;
//using ISAAR.MSolve.FEM;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.MSolve.Discretization.Entities;
using System.Data;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Transient;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.Environments;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using System.IO;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Constitutive;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using IntelMKL.LP64;
using MGroup.MSolve.Discretization.BoundaryConditions;
//using ISAAR.MSolve.PreProcessor;
//using ISAAR.MSolve.Discretization.Commons;
//using ISAAR.MSolve.Discretization.FreedomDegrees;
//using ISAAR.MSolve.Discretization.Integration.Quadratures;
//using ISAAR.MSolve.FEM.Elements;
//using ISAAR.MSolve.FEM.Entities;
//using ISAAR.MSolve.Logging;
//using ISAAR.MSolve.Materials;
//using ISAAR.MSolve.Problems;
//using ISAAR.MSolve.Solvers.Direct;
//using ISAAR.MSolve.PreProcessor.Materials;
//using ISAAR.MSolve.FEM.Entities.TemporalFunctions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
    public class VonMisesNonLinearCyclicTestExample
    {


        public static double startX = 0.0;
        public static double startY = 0.0;
        public static double startZ = 0.0;
        public static double LengthX = 1.0;
        public static double LengthY = 1.0;
        public static double LengthZ = 1.0;
        int nodeID = 1;
        public static double hx = 2.0;
        public static double hy = 2.0;
        public static double hz = 2.0;
        public static int imax = (int)Math.Truncate(hx / LengthX) + 1;
        public static int jmax = (int)Math.Truncate(hy / LengthY) + 1;
        public static int kmax = (int)Math.Truncate(hz / LengthZ) + 1;

        public static Model MakeHexaSoil(Model model)
        {
            // xreiazetai na rythmizei kaneis ta megethi me auto to configuration
            int nodeID = 1;
            for (int l = 0; l < kmax; l++)
            {
                for (int k = 0; k < jmax; k++)
                {
                    for (int j = 0; j < imax; j++)
                    {
                        model.NodesDictionary.Add(nodeID, new Node(nodeID, startX + j * LengthX, startY + k * LengthY, startZ + l * LengthZ));
                        nodeID++;
                    }
                }
            }
            //nodeID = 1;
            var constraints = new List<INodalDisplacementBoundaryCondition>();
            constraints.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationZ, amount: 0d));


            ElasticMaterial3D material1 = new ElasticMaterial3D(800000d, 1.0d / 3.0d);

            //ContinuumElement3DFactory e1;

            int IDhelp = 1;
            var elementCounter = 1;
            for (int ii = 1; ii < model.NodesDictionary.Count + 1; ii++)
            {
                var nodecheck = model.NodesDictionary[ii];
                if (nodecheck.X != hx && nodecheck.Y != hy && nodecheck.Z != hz)
                {
                    const int gpNo = 8;
                    //var initialStresses = new double[6];
                    var element1Nodes = new INode[8];

                    element1Nodes[0] = model.NodesDictionary[ii];
                    element1Nodes[1] = model.NodesDictionary[ii + 1];
                    element1Nodes[2] = model.NodesDictionary[ii + 1 + imax];
                    element1Nodes[3] = model.NodesDictionary[ii + imax];
                    element1Nodes[4] = model.NodesDictionary[ii + jmax * imax];
                    element1Nodes[5] = model.NodesDictionary[ii + jmax * imax + 1];
                    element1Nodes[6] = model.NodesDictionary[ii + jmax * imax + 1 + imax];
                    element1Nodes[7] = model.NodesDictionary[ii + jmax * imax + imax];


                    var nodeCoordinates = new double[8, 3];
                    for (int i = 0; i < 8; i++)
                    {
                        nodeCoordinates[i, 0] = element1Nodes[i].X;
                        nodeCoordinates[i, 1] = element1Nodes[i].Y;
                        nodeCoordinates[i, 2] = element1Nodes[i].Z;
                    }


                    var gaussPointMaterials = new VonMises3DNonLinearHardening[8];
                    double young = 800000d;
                    double poisson = 1.0d / 3.0d;
                    double syield = 5000d;
                    for (int i = 0; i < gpNo; i++)
                        gaussPointMaterials[i] = new VonMises3DNonLinearHardening(young, poisson, syield);




                    //var gaussPointMaterials = new VonMisesMaterial3D[8];
                    //double youngModulus = 800000d;
                    //double poissonRatio = 1.0d / 3.0d;
                    //double yieldStress = 5000d;
                    //double hardeningRatio = 100000d;
                    //for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new VonMisesMaterial3D(youngModulus, poissonRatio, yieldStress, hardeningRatio);

                    var elementFactory = new ContinuumElement3DFactory(new VonMises3DNonLinearHardening(800000d, 1.0d / 3.0d, 5000d), new TransientAnalysisProperties(1, 0, 0));
                    var element = elementFactory.CreateElement(CellType.Hexa8,
                        new INode[]
                        {
                            element1Nodes[6],
                            element1Nodes[7],
                            element1Nodes[4],
                            element1Nodes[5],
                            element1Nodes[2],
                            element1Nodes[3],
                            element1Nodes[0],
                            element1Nodes[1]

                        });

                    {
                        int ID = IDhelp;
                    };
                    IDhelp++;
                    int subdomainID = 1;
                    element.ID = elementCounter;
                    elementCounter += 1;
                    //element.Node
                    model.ElementsDictionary.Add(element.ID, element);
                    model.SubdomainsDictionary[subdomainID].Elements.Add(element);
                }
            };
            #region initialloads
            //DOFType doftype1;
            //doftype1 = new DOFType();
            var emptyloads1 = new List<INodalLoadBoundaryCondition>();

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, emptyloads1));

            return model;

            
        }

        public static double TimeFunctionPeriodicLoad(double t, double amount)
        {


            double[] time = new double[401] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400 };

          

            double[] timeFunctionValues = new double[401] { 0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 0.30, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.40, 0.41, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.50, 0.51, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59, 0.60, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69, 0.70, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79, 0.80, 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 0.88, 0.89, 0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 1, 0.99, 0.98, 0.97, 0.96, 0.95, 0.94, 0.93, 0.92, 0.91, 0.9, 0.89, 0.88, 0.87, 0.86, 0.85, 0.84, 0.83, 0.82, 0.81, 0.8, 0.79, 0.78, 0.77, 0.76, 0.75, 0.74, 0.73, 0.72, 0.71, 0.7, 0.69, 0.68, 0.67, 0.66, 0.65, 0.64, 0.63, 0.62, 0.61, 0.6, 0.59, 0.58, 0.57, 0.56, 0.55, 0.54, 0.53, 0.52, 0.51, 0.5, 0.49, 0.48, 0.47, 0.46, 0.45, 0.44, 0.43, 0.42, 0.41, 0.4, 0.39, 0.38, 0.37, 0.36, 0.35, 0.34, 0.33, 0.32, 0.31, 0.3, 0.29, 0.28, 0.27, 0.26, 0.25, 0.24, 0.23, 0.22, 0.21, 0.2, 0.19, 0.18, 0.17, 0.16, 0.15, 0.14, 0.13, 0.12, 0.11, 0.1, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0, -0.01, -0.02, -0.03, -0.04, -0.05, -0.06, -0.07, -0.08, -0.09, -0.1, -0.11, -0.12, -0.13, -0.14, -0.15, -0.16, -0.17, -0.18, -0.19, -0.20, -0.21, -0.22, -0.23, -0.24, -0.25, -0.26, -0.27, -0.28, -0.29, -0.30, -0.31, -0.32, -0.33, -0.34, -0.35, -0.36, -0.37, -0.38, -0.39, -0.40, -0.41, -0.42, -0.43, -0.44, -0.45, -0.46, -0.47, -0.48, -0.49, -0.50, -0.51, -0.52, -0.53, -0.54, -0.55, -0.56, -0.57, -0.58, -0.59, -0.60, -0.61, -0.62, -0.63, -0.64, -0.65, -0.66, -0.67, -0.68, -0.69, -0.70, -0.71, -0.72, -0.73, -0.74, -0.75, -0.76, -0.77, -0.78, -0.79, -0.80, -0.81, -0.82, -0.83, -0.84, -0.85, -0.86, -0.87, -0.88, -0.89, -0.9, -0.91, -0.92, -0.93, -0.94, -0.95, -0.96, -0.97, -0.98, -0.99, -1, -0.99, -0.98, -0.97, -0.96, -0.95, -0.94, -0.93, -0.92, -0.91, -0.9, -0.89, -0.88, -0.87, -0.86, -0.85, -0.84, -0.83, -0.82, -0.81, -0.8, -0.79, -0.78, -0.77, -0.76, -0.75, -0.74, -0.73, -0.72, -0.71, -0.7, -0.69, -0.68, -0.67, -0.66, -0.65, -0.64, -0.63, -0.62, -0.61, -0.6, -0.59, -0.58, -0.57, -0.56, -0.55, -0.54, -0.53, -0.52, -0.51, -0.5, -0.49, -0.48, -0.47, -0.46, -0.45, -0.44, -0.43, -0.42, -0.41, -0.4, -0.39, -0.38, -0.37, -0.36, -0.35, -0.34, -0.33, -0.32, -0.31, -0.3, -0.29, -0.28, -0.27, -0.26, -0.25, -0.24, -0.23, -0.22, -0.21, -0.2, -0.19, -0.18, -0.17, -0.16, -0.15, -0.14, -0.13, -0.12, -0.11, -0.1, -0.09, -0.08, -0.07, -0.06, -0.05, -0.04, -0.03, -0.02, -0.01, 0 };

            for (int i1 = 0; i1 < time.Length - 1; i1++)
            {


                //for (int i = 0; i < 100; i++)
                //{
                //    time[i] = 0.01 * (1 + i);
                //}
                //for (int i = 100; i < 200; i++)
                //{
                //    time[i] = 1 - 0.01 * (i - 100 + 1);
                //}
                //for (int i = 200; i < 300; i++)
                //{
                //    time[i] = -0.01 * (1 + i - 200);
                //}
                //for (int i = 300; i < 400; i++)
                //{
                //    time[i] = -(1 - 0.01 * (i - 300 + 1));

                //}




                if (t == 0)
                {
                    double endiameshTimeFuncValue = time[0];
                    return endiameshTimeFuncValue * amount;
                }

                if ((time[i1] > 0) && (t <= time[i1 + 1]) && (time[i1 + 1] <= 100) && timeFunctionValues[i1] < 1)
                {


                    return timeFunctionValues[i1] * amount;
                }

                if ((time[i1] > 101) && (t <= time[i1 + 1]) && (time[i1 + 1] <= 300) && timeFunctionValues[i1] < -1)
                {


                    return timeFunctionValues[i1] * amount;
                }

                if ((time[i1] > 301) && (t <= time[i1 + 1]) && (time[i1 + 1] <= 400) && timeFunctionValues[i1] < 0)
                {

                    return timeFunctionValues[i1] * amount;
                }


            }



            //if ((time[i1] < t) && (t <= time[i1 + 1]))
            //{
            //    double klisi = (timeFunctionValues[i1 + 1] - timeFunctionValues[i1]) / (time[i1 + 1] - time[i1]);
            //    double dt = t - time[i1];
            //    double deltaTimeFunction = klisi * dt;
            //    double endiameshTimeFuncValue = timeFunctionValues[i1] + deltaTimeFunction;
            //    return endiameshTimeFuncValue * amount;
            //}




            throw new Exception("time out of range");

        }

        public static void AddPeriodicTransientLoad(Model model)
        {


            var loads = new List<INodalLoadBoundaryCondition>();

            loads.Add(new NodalLoad(model.NodesDictionary[19], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[22], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[25], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[20], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[23], StructuralDof.TranslationZ, amount: 1 * (-15000.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[26], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[21], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[24], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[27], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));


            var Constraints1 = new List<INodalDisplacementBoundaryCondition>();


            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationZ, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationX, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationY, amount: 0d));
            Constraints1.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationZ, amount: 0d));







            var boundaryLoadConditionSet = new StructuralBoundaryConditionSet(Constraints1, loads);

            var transientLoadandBCs = new StructuralTransientBoundaryConditionSet(new List<IBoundaryConditionSet<IStructuralDofType>>() { boundaryLoadConditionSet }, TimeFunctionPeriodicLoad);

            model.BoundaryConditions.Add(transientLoadandBCs);
            
        }

        #endregion

    

        public static IReadOnlyList<double[]> GetExpectedDisplacements()
        {
            return new double[][]
            {
                 new[] {-0.0251993148614215},


            };
        }
    }

}
        




