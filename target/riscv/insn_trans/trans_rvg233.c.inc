static bool trans_dma(DisasContext *ctx, arg_dma *a)
{
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv gran = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_dma(tcg_env, dst, src, gran);
    return true;
}