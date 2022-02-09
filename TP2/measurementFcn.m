function z = measurementFcn(x, varargin)
    cartState = filterToCartState(x);
    z = cvmeas(cartState,varargin{:});
end