namespace KalmanFilter

module DataHandling =
    open System
    open MathNet.Numerics.LinearAlgebra
    open FSharp.Data
    open FSharp.Data.CsvExtensions
    open MathNet.Numerics.LinearAlgebra.DenseMatrix
    open MathNet.Numerics.LinearAlgebra.Matrix

    // Create type for accessing Yahoo Finance
    let [<Literal>] msftUrl = "https://www.quandl.com/api/v3/datasets/WIKI/FB/data.csv?api_key=Qkzr33_5458Regb6Z9Qv&order=asc"
    type StockData = CsvProvider<msftUrl>
    type Series = String*StockData
    let apikey = "Qkzr33_5458Regb6Z9Qv"
    

    /// Helper function that returns URL for required stock data
    let urlFor ticker order apikey (startDate:System.DateTime) (endDate:System.DateTime) = 
        let root = "https://www.quandl.com/api/v3/datasets/WIKI"
        sprintf "%s/%s/data.csv?api_key=%s&order=%s&start_date=%d-%02d-%02d&end_date=%d-%02d-%02d"
            root ticker apikey order
            startDate.Year startDate.Month startDate.Day
            endDate.Year endDate.Month endDate.Day

    /// Returns stock data for a given ticker name and date range
    let stockData ticker startDate endDate =
        StockData.Load(urlFor ticker "asc" apikey startDate endDate)

    // Names of stock market indicators
    let indicatorNames = [|"^AORD"; "^FCHI"; "^FTSE"; "^GSPTSE"; "^MERV"; "^MXX"; "^NDX"|]
    let stockNames = [|"MSFT"; "AAPL"; "FB"; "AMZN"; "DIS"; "CSCO"; "NKE"|]

    let getSeries names =
         [| for name in names ->
            name, stockData name (DateTime(2011,1,1)) DateTime.Now
        |]

    let lastColumns nColumns (matrix:Matrix<float>) =
        let columnCount = matrix.ColumnCount
        matrix.[0 .. , columnCount - nColumns ..]

    let prepareData (stocksData:Series[]) =
        let names = Array.map (fun (name, _) -> name) stocksData
        // Dates when data for all indices are available
        let commonDates = 
            [ for name, index in stocksData -> 
                // Return a set with available dates for the current index
                set [ for item in index.Rows -> item.Date ] ]
            |> Set.intersectMany

        // Create a matrix with historical data for available dates
        [ for name, index in stocksData ->
            [ for item in index.Rows do
                if commonDates.Contains item.Date
                then yield (float item.Close)
            ]
        ]
        |> matrix
        
    let prepareData2 (indexData:CsvFile) (names:string seq) =
        [ for row in indexData.Rows ->
            [ for name in names do
                yield (float <| row.GetColumn name)
            ]
        ]
        |> matrix
        |> transpose

    let normalize (low:float) (high:float) (data:Matrix<float>) =
        [ for rowI in [0 .. data.RowCount - 1] do
            let row = data.Row(rowI)
            let min, max = row.Minimum(), row.Maximum()
            yield
                row.Map(fun v -> (high - low) * (v - min)/(max - min) + low)
        ] |> ofRows
        
    let splitData (historicalData:Matrix<float>) =
        // Split the data into training set and validation set
        let trainingT = historicalData.ColumnCount * 2 / 3
        let trainingData =
            historicalData.[0 .. , .. trainingT]
        let validationData =
            historicalData.[0 .. , trainingT + 1 .. ]
        trainingData, validationData