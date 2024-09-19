import random

def generate_cvrp_instance(num_nodes, num_vehicles, capacity, max_coord=10000, max_demand=25, output_file="input_large_5.txt"):
    """
    Gera uma instância de CVRP com o número de nós, veículos e capacidade fornecidos.

    Args:
        num_nodes (int): Número total de nós (incluindo o depósito).
        num_vehicles (int): Número de veículos.
        capacity (int): Capacidade de cada veículo.
        max_coord (int): Valor máximo das coordenadas x e y (default 100).
        max_demand (int): Demanda máxima de cada nó (default 30).
        output_file (str): Nome do arquivo de saída (default 'cvrp_instance.txt').
    """

    with open(output_file, 'w') as file:
        # Header
        file.write(f"NAME : A-n{num_nodes}-k{num_vehicles}\n")
        file.write(f"COMMENT : (Generated, No of trucks: {num_vehicles}, Optimal value: Unknown)\n")
        file.write("TYPE : CVRP\n")
        file.write(f"DIMENSION : {num_nodes}\n")
        file.write("EDGE_WEIGHT_TYPE : EUC_2D\n")
        file.write(f"CAPACITY : {capacity}\n")

        # Node coordinates
        file.write("NODE_COORD_SECTION\n")
        for i in range(1, num_nodes + 1):
            x = random.randint(0, max_coord)
            y = random.randint(0, max_coord)
            file.write(f" {i} {x} {y}\n")

        # Demands
        file.write("DEMAND_SECTION\n")
        file.write(f" 1 0\n")  # Depot has a demand of 0
        for i in range(2, num_nodes + 1):
            demand = random.randint(1, max_demand)
            file.write(f" {i} {demand}\n")

        # Depot section
        file.write("DEPOT_SECTION\n")
        file.write(" 1\n")
        file.write(" -1\n")

        # End of file
        file.write("EOF\n")

    print(f"CVRP instance with {num_nodes} nodes and {num_vehicles} vehicles generated in {output_file}.")

# Example usage:
generate_cvrp_instance(num_nodes=3000, num_vehicles=900, capacity=100, output_file="input_large_5.txt")
