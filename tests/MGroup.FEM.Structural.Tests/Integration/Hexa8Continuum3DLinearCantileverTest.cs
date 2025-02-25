using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Tests.Commons;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public static class Hexa8Continuum3DLinearCantileverTest
	{
		[Fact]
		private static void RunTest()
		{
			var model = Hexa8Continuum3DLinearCantileverExample.CreateModel();
			var computedDisplacements = SolveModel(model);
			Assert.True(Utilities.AreDisplacementsSame(Hexa8Continuum3DLinearCantileverExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 1e-9));
		}

		private static IncrementalDisplacementsLog SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel, solver);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, problem, numIncrements: 2)
			{
				ResidualTolerance = 1E-8,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1
			};
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, problem, loadControlAnalyzer);

			loadControlAnalyzer.IncrementalDisplacementsLog = new IncrementalDisplacementsLog(
				new List<(INode node, IDofType dof)>()
				{
					(model.NodesDictionary[5], StructuralDof.TranslationX),
					(model.NodesDictionary[8], StructuralDof.TranslationZ),
					(model.NodesDictionary[12], StructuralDof.TranslationZ),
					(model.NodesDictionary[16], StructuralDof.TranslationZ),
					(model.NodesDictionary[20], StructuralDof.TranslationZ)
				}, algebraicModel
			);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return loadControlAnalyzer.IncrementalDisplacementsLog;
		}
	}

}
