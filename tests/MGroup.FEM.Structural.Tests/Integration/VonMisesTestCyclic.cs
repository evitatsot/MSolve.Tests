﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.LinearAlgebra;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.NonLinear;
//using ISAAR.MSolve.Analyzers.NonLinear;
using MGroup.LinearAlgebra.Vectors;
//using ISAAR.MSolve.Analyzers.NonLinear;
//using ISAAR.MSolve.Discretization;
using MGroup.NumericalAnalyzers.Discretization;
using MGroup.FEM;
using MGroup.FEM.Structural;
using MGroup.FEM.Structural.Tests;
using MGroup.FEM.Structural.Tests.Integration;
using MGroup.Solvers;
//using ISAAR.MSolve.FEM;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.MSolve.Discretization.Entities;
using System.Data;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Solution;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;
using MGroup.Environments;
using System.IO;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.FEM.Structural.Tests.Commons;
using Xunit;
using Utilities = MGroup.FEM.Structural.Tests.Commons.Utilities;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.AlgebraicModel;
using IntelMKL.LP64;
//using ISAAR.MSolve.PreProcessor;
//using ISAAR.MSolve.PreProcessor.Materials;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.IO;
//using System.Threading;
//using System.Threading.Tasks;
//using ISAAR.MSolve.Analyzers.Interfaces;
using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Commons;
using Xunit;
using System.Linq;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Commons;
using Xunit;
using MGroup.Solvers;


namespace MGroup.FEM.Structural.Tests.Integration
{
    public class VonMisesTestCyclic
    {

        private const int subdomainID = 0;
        #region Mgroupstuff
        #region readwritemethods
        public static void ReadData(string DataFileName, out double[] array)
        {
            string dataLine;
            string[] dataFields;
            string[] numSeparators1 = { ":" };
            string[] numSeparators2 = { " " };
            StreamReader rStream;
            rStream = File.OpenText(DataFileName);
            int dim = 1;
            dataLine = rStream.ReadLine();
            dataFields = dataLine.Split(numSeparators1, StringSplitOptions.RemoveEmptyEntries);
            dim = int.Parse(dataFields[0]);
            array = new double[dim];
            for (int i = 0; i < dim; i++)
            {
                dataLine = rStream.ReadLine();
                dataFields = dataLine.Split(numSeparators1, StringSplitOptions.RemoveEmptyEntries);
                array[i] = double.Parse(dataFields[0]);
            }
            rStream.Close();

        }
        public static void WriteData(double[] array, int identifier)
        {
            string filename, dataLine;
            // The identifier is for telling if you want to write the whole array (1) or the last element (0) (for example the whole displacement curve or the last increment)
            // To insert spaces, use the simple space character " ", not tabs (i.e. "\t"). 
            // the editors do not 'interpret' the tabs in the same way, 
            // so if you open the file with different editors can be a mess.
            //string spaces1 = "        ";
            //string spaces2 = "              ";

            // format specifier to write the real numbers
            string fmtSpecifier = "{0: 0.0000E+00;-0.0000E+00}";

            StreamWriter wStream;
            filename = "displacements.txt";
            wStream = File.CreateText(filename);
            if (identifier == 1)
            {
                for (int i = 0; i < array.GetLength(0); i++)
                {
                    dataLine = String.Format(fmtSpecifier, array[i]);
                    wStream.WriteLine(dataLine);
                }
                wStream.Close();
            }
            else
            {
                dataLine = String.Format(fmtSpecifier, array[array.GetLength(0) - 1]);
                wStream.WriteLine(dataLine);
                wStream.Close();
            }
        }
        public static void WriteData(double[] array, int identifier, string filename)
        {
            string dataLine;
            // The identifier is for telling if you want to write the whole array (1) or the last element (0) (for example the whole displacement curve or the last increment)
            // To insert spaces, use the simple space character " ", not tabs (i.e. "\t"). 
            // the editors do not 'interpret' the tabs in the same way, 
            // so if you open the file with different editors can be a mess.
            //string spaces1 = "        ";
            //string spaces2 = "              ";

            // format specifier to write the real numbers
            string fmtSpecifier = "{0: 0.0000E+00;-0.0000E+00}";

            StreamWriter wStream;

            wStream = File.CreateText(filename);
            if (identifier == 1)
            {
                for (int i = 0; i < array.GetLength(0); i++)
                {
                    dataLine = String.Format(fmtSpecifier, array[i]);
                    wStream.WriteLine(dataLine);
                }
                wStream.Close();
            }
            else
            {
                dataLine = String.Format(fmtSpecifier, array[array.GetLength(0) - 1]);
                wStream.WriteLine(dataLine);
                wStream.Close();
            }
        }
        public static void ReadMatrixData(string DataFileName, out double[,] array)
        {
            string dataLine;
            string[] dataFields;
            string[] numSeparators1 = { ":" };
            string[] numSeparators2 = { " " };
            StreamReader rStream;
            rStream = File.OpenText(DataFileName);
            int dim = 1;
            int dim1 = 1;
            dataLine = rStream.ReadLine();
            dataFields = dataLine.Split(numSeparators1, StringSplitOptions.RemoveEmptyEntries);
            dim = int.Parse(dataFields[0]);
            dataLine = rStream.ReadLine();
            dataFields = dataLine.Split(numSeparators1, StringSplitOptions.RemoveEmptyEntries);
            dim1 = int.Parse(dataFields[0]);
            double[,] array1 = new double[dim, dim1];
            for (int i = 0; i < dim; i++)
            {
                dataLine = rStream.ReadLine();
                dataFields = dataLine.Split(numSeparators1, StringSplitOptions.RemoveEmptyEntries);
                for (int j = 0; j < dim1; j++)
                {
                    array1[i, j] = double.Parse(dataFields[j]);
                }
            }
            rStream.Close();
            array = array1;
        }
        public static double[] ReadMatrixDataPartially(double[,] Matrix, int rowbegin, int rowend, int colbegin, int colend)
        {
            var array = new double[(rowend + 1 - rowbegin) * (colend + 1 - colbegin)];
            var k = 0;
            for (int i = rowbegin; i < rowend + 1; i++)
            {
                for (int j = colbegin; j < colend + 1; j++)
                {
                    array[k] = Matrix[i, j];
                    k++;
                }
            }
            return array;
        }
        public static int NR_steps = 1;
        private static double timestep = 1; // those are overwritten for the periodic example
        private static double totalTime = 400; // those are overwritten for the periodic example

        #endregion
        #region solvermethods



        [Fact]
        private static void RunTest()
        {
            
            Model model = new Model();
            model.SubdomainsDictionary.Add(1, new Subdomain(1));

            VonMisesNonLinearCyclicTestExample.MakeHexaSoil(model);
            //model.ConnectDataStructures();
            VonMisesNonLinearCyclicTestExample.AddPeriodicTransientLoad(model);

            var computedDisplacements = SolveModel(model);
            timestep = 1; totalTime=400;


        Assert.True(Utilities.AreDisplacementsSame(VonMisesNonLinearCyclicTestExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 2.6e-2));

        }


        private static IncrementalDisplacementsLog SolveModel(Model model)
        {
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemStructural(model, algebraicModel);

            var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: NR_steps)
            {
                ResidualTolerance = 1E-5,
                MaxIterationsPerIncrement = 100,
                NumIterationsForMatrixRebuild = 1
            };
            var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
            //var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, loadControlAnalyzer);

            var dynamicAnalyzerBuilder=new PseudoTransientAnalyzer.Builder(algebraicModel, problem, loadControlAnalyzer, timeStep: timestep, totalTime: totalTime);
            //var dynamicAnalyzerBuilder = new PseudoTransientAnalyzer.Builder(algebraicModel, problem, loadControlAnalyzer, totalTime);
            PseudoTransientAnalyzer parentAnalyzer = dynamicAnalyzerBuilder.Build();
            //var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();



            loadControlAnalyzer.IncrementalDisplacementsLog = new IncrementalDisplacementsLog(
                new List<(INode node, IDofType dof)>()
                {

                    (model.NodesDictionary[23], StructuralDof.TranslationZ)
                }, algebraicModel
            );

            //dynamicAnalyzer.Initialize();
            //dynamicAnalyzer.Solve();

            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();

            return loadControlAnalyzer.IncrementalDisplacementsLog;

            #endregion

        }

    #endregion

    }
}
